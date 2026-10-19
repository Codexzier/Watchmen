# =====================================================================================
#  Watchmen – Ablaufsteuerung (Zustandsmaschine)
# =====================================================================================
#  Zustände:
#    boot                  Warten auf den Sketch (MCU)
#    kalibrierung          Kalibrierung 1: Bereiche der Servos/Drehachse auf der Webseite
#    kalibrierung2         Kalibrierung 2: Richtungen automatisch mit der Kamera messen
#    kalibrierung2_manuell Kalibrierung 2 hat nicht geklappt → Richtungen von Hand
#    kalibriert            Kalibrierung abgeschlossen: 5 s beide LEDs grün
#    referenzfahrt         nach dem Einschalten: Drehachse fährt den Kontaktschalter an
#    aktiv                 Erkennung läuft (Muster > Person > Auto), Nachführung der Kamera
#    ruhemodus             1 Minute nichts erkannt: Kamera aus, LED 2 blinkt orange
#    fehler                z.B. Referenzfahrt fehlgeschlagen
#  Ohne Motoren (A0 = 0 V) gibt es nur "aktiv"/"ruhemodus" ohne Bewegung.
# =====================================================================================

import queue                                                  # Befehlswarteschlange (Webseite → Hauptschleife)
import threading                                              # Sperren
import time                                                   # Zeitmessung
from collections import deque                                 # Liste der letzten Meldungen

from arduino.app_utils import Logger                          # Protokollierung

import mcu as mcu_mod                                         # Konstanten der Sketch-Schnittstelle
from calibration import AutoCalibration                       # Kalibrierung 2 (automatisch)
from radar import parse_targets, is_wake_target, choose_person  # Radar-Auswertung
from tracker import Tracker, manual_fractions                 # Nachführung
from vision import orient, to_raw_point, to_raw_box           # Bildlage-Hilfen

logger = Logger("watchmen")                                   # Logger der Anwendung

STATE_BOOT = "boot"                                           # Warten auf den Sketch
STATE_CALIB = "kalibrierung"                                  # Kalibrierung 1
STATE_CALIB2 = "kalibrierung2"                                # Kalibrierung 2 automatisch
STATE_CALIB2_MANUAL = "kalibrierung2_manuell"                 # Kalibrierung 2 von Hand
STATE_CALIB_DONE = "kalibriert"                               # 5 s grün
STATE_HOMING = "referenzfahrt"                                # Referenzfahrt
STATE_ACTIVE = "aktiv"                                        # Erkennung läuft
STATE_SLEEP = "ruhemodus"                                     # Ruhemodus
STATE_ERROR = "fehler"                                        # Fehler

STATE_TEXT = {                                                # Beschreibungen für die Webseite
    STATE_BOOT: "Warte auf Mikrocontroller",                  # boot
    STATE_CALIB: "Nicht kalibriert – Kalibrierung 1 (Bereiche einstellen)",  # kalibrierung
    STATE_CALIB2: "Kalibrierung 2 läuft (Richtungen werden gemessen)",  # kalibrierung2
    STATE_CALIB2_MANUAL: "Kalibrierung 2 – bitte Richtungen eintragen und speichern",  # manuell
    STATE_CALIB_DONE: "Kalibrierung abgeschlossen",           # kalibriert
    STATE_HOMING: "Referenzfahrt zum Kontaktschalter",        # referenzfahrt
    STATE_ACTIVE: "Objekterkennung aktiv",                    # aktiv
    STATE_SLEEP: "Ruhemodus (Kamera aus)",                    # ruhemodus
    STATE_ERROR: "Fehler",                                    # fehler
}

CALIB_STATES = (STATE_CALIB, STATE_CALIB2, STATE_CALIB2_MANUAL)  # Zustände, in denen kalibriert wird

OFF = 0x000000                                                # LED aus
RED = 0xFF0000                                                # Rot
GREEN = 0x00FF00                                              # Grün
BLUE = 0x0000FF                                               # Blau
ORANGE = 0xFF6000                                             # Orange
WHITE = 0xFFFFFF                                              # Weiß
CYAN = 0x00FFFF                                               # Türkis

LABEL_PRIORITY = ("marker", "person", "car")                  # Muster vor Person vor Auto

SETTINGS_KEYS = (                                             # Einstellungen, die die Webseite ändern darf
    "sleep_timeout_s", "radar_wake_distance_mm", "radar_wake_min_speed_cms", "radar_wake_frames",  # Ruhemodus
    "vibration_threshold_mg", "detection_confidence", "person_labels", "car_labels",  # Schwellwerte, Klassen
    "marker_dictionary", "marker_id", "hold_time_s", "mount_mode", "mpu_up_axis",  # Muster, Montage
    "track_gain", "track_deadband", "track_interval_s", "text_car", "text_person", "text_marker",  # Nachführung, Texte
    "led_brightness", "radar_mirror_x", "radar_max_range_mm", "camera_source", "camera_width",  # Ausgaben, Radar, Kamera
    "camera_height", "camera_fps", "camera_hfov_deg", "camera_vfov_deg", "stepper_max_speed",  # Kamera, Motor
    "servo_speed_dps", "stepper_steps_per_rev",               # Motoren
)

CALIB1_KEYS = (                                               # Werte, die Kalibrierung 1 speichert
    "tilt_min", "tilt_max", "tilt_level", "radar_min", "radar_max", "radar_level",  # Servos
    "radar_comp_factor", "radar_comp_enabled", "pan_min", "pan_max", "pan_center",  # Radar-Ausgleich, Drehachse
    "stepper_invert", "servo_us_min", "servo_us_max", "homing_max_steps",  # Richtung, Impulse, Suchweg
)


class Controller:                                             # die zentrale Ablaufsteuerung
    def __init__(self, cfg, mcu, vision, ui):                 # Konstruktor
        self.cfg = cfg                                        # Konfiguration
        self.mcu = mcu                                        # Sketch-Schnittstelle
        self.vision = vision                                  # Kamera und Erkennung
        self.ui = ui                                          # Webseite
        self.tracker = Tracker()                              # Nachführung
        self.commands = queue.Queue()                         # Befehle von der Webseite
        self.state = STATE_BOOT                               # Startzustand
        self.state_since = time.monotonic()                   # seit wann im Zustand
        self.error_text = ""                                  # Fehlerbeschreibung
        self.flipped = False                                  # Deckenmontage erkannt?
        self.calib2 = None                                    # Thread der Kalibrierung 2
        self.calib2_info = {"status": "", "message": "", "result": None}  # Fortschritt/Ergebnis Kalibrierung 2
        self.last_detection = time.monotonic()                # letzte Erkennung (für Ruhemodus)
        self.last_seen = {k: 0.0 for k in LABEL_PRIORITY}     # letzte Sichtung je Art
        self.current_label = None                             # aktuell angezeigte Art (None = noch nichts gesendet)
        self.last_detections = []                             # Erkennungen des letzten Bildes (für Webseite)
        self.radar_targets = []                               # gültige Radar-Ziele
        self.radar_time = 0.0                                 # Zeitpunkt der Radar-Ziele
        self.radar_selected = None                            # Radar-Ziel der verfolgten Person
        self._radar_seq = 0                                   # zuletzt verarbeitete Radar-Meldung
        self._radar_sent = 0.0                                # Zeitpunkt der letzten Radar-Ausgabe
        self._wake_frames = 0                                 # Radar-Weckzähler
        self._vib_seq = 0                                     # zuletzt verarbeitete Vibration
        self._wake_reason = ""                                # Grund für das Aufwachen
        self._motors_prev = None                              # letzter Zustand von A0
        self._config_dirty = True                             # Konfiguration muss zum Sketch
        self._config_sent = 0.0                               # Zeitpunkt der letzten Übertragung
        self._published = 0.0                                 # Zeitpunkt der letzten Status-Ausgabe
        self._placeholder_time = 0.0                          # Zeitpunkt des letzten Ersatzbildes
        self._fps_count = 0                                   # Bilder seit letzter Messung
        self._fps_time = time.monotonic()                     # Beginn der Messung
        self.fps = 0.0                                        # Bilder pro Sekunde
        self.events = deque(maxlen=40)                        # letzte Meldungen für die Webseite
        self.state_snapshot = {}                              # letzter veröffentlichter Zustand
        self._lock = threading.Lock()                         # schützt state_snapshot

    # ================================================================ Hilfen ==========
    def log(self, text):                                      # Meldung protokollieren und an die Webseite
        logger.info(text)                                     # ins Protokoll
        self.events.appendleft({"t": time.strftime("%H:%M:%S"), "text": text})  # in die Liste (neueste zuerst)

    def calibrated(self):                                     # ist das System vollständig kalibriert?
        return bool(self.cfg["calib1_done"] and self.cfg["calib2_done"])  # beide Kalibrierungen erledigt

    def motion_enabled(self, st):                             # dürfen sich die Motoren bewegen (Nachführung)?
        return bool(st["online"] and st["motors_present"] and st["homed"] and self.calibrated())  # alle Bedingungen

    def send(self, name, data):                               # Nachricht an alle Webseiten-Clients
        try:                                                  # Fehler abfangen (Server evtl. noch nicht bereit)
            self.ui.send_message(name, data)                  # senden
        except Exception as e:                                # Fehler
            logger.debug(f"Senden an Webseite fehlgeschlagen: {e}")  # nur Debug

    def send_config(self):                                    # Konfiguration an die Webseite
        self.send("config", self.cfg.snapshot())              # vollständige Kopie senden

    # ================================================================ Hauptschleife ===
    def loop_once(self):                                      # wird von App.run() immer wieder aufgerufen
        try:                                                  # Fehler dürfen die App nicht beenden
            self._step()                                      # ein Durchlauf
        except Exception as e:                                # unerwarteter Fehler
            logger.exception(f"Fehler in der Hauptschleife: {e}")  # protokollieren
            time.sleep(0.5)                                   # kurz warten, dann weiter

    def _step(self):                                          # ein Durchlauf der Hauptschleife
        now = time.monotonic()                                # aktuelle Zeit
        self._process_commands()                              # Befehle der Webseite ausführen
        st = self.mcu.get_status()                            # Zustand des Sketches
        self._process_events()                                # Ereignisse des Sketches
        self._sync_config(st, now)                            # Konfiguration zum Sketch übertragen
        self._update_orientation(st)                          # Decken- oder Normalmontage
        self._update_radar(now)                               # Radar-Ziele auswerten
        self._update_vibration()                              # Vibrationen auswerten
        self._check_motors(st)                                # Motoren an/ab gesteckt?
        handler = {                                           # passende Funktion je Zustand
            STATE_BOOT: self._run_boot,                       # boot
            STATE_CALIB: self._run_preview,                   # kalibrierung (nur Kamerabild)
            STATE_CALIB2: self._run_preview,                  # kalibrierung2 (Thread arbeitet)
            STATE_CALIB2_MANUAL: self._run_preview,           # kalibrierung2_manuell
            STATE_CALIB_DONE: self._run_calib_done,           # kalibriert (5 s grün)
            STATE_HOMING: self._run_homing,                   # referenzfahrt
            STATE_ACTIVE: self._run_active,                   # aktiv
            STATE_SLEEP: self._run_sleep,                     # ruhemodus
            STATE_ERROR: self._run_preview,                   # fehler
        }[self.state]                                         # Funktion auswählen
        handler(now, st)                                      # ausführen
        self._publish(now, st)                                # Zustand an die Webseite

    # ================================================================ Zustandswechsel =
    def set_state(self, new_state, reason=""):                # wechselt den Zustand
        old = self.state                                      # alter Zustand
        if old == STATE_CALIB2 and new_state != STATE_CALIB2 and self.calib2 is not None:  # Kalibrierung 2 verlassen
            self.calib2.cancel()                              # laufenden Thread abbrechen
            self.calib2 = None                                # vergessen
        if old in CALIB_STATES and new_state not in CALIB_STATES:  # Kalibrierung verlassen
            self._config_dirty = True                         # gespeicherte Konfiguration erneut senden (Testwerte verwerfen)
        self.state = new_state                                # neuen Zustand setzen
        self.state_since = time.monotonic()                   # Zeitpunkt merken
        self.log(f"Zustand: {STATE_TEXT[new_state]}" + (f" ({reason})" if reason else ""))  # Meldung
        st = self.mcu.get_status()                            # aktueller Status des Sketches
        if new_state in CALIB_STATES:                         # nicht (fertig) kalibriert
            self.vision.start_camera()                        # Kamerabild zum Einstellen
            self.mcu.led(2, RED, 500)                         # beide LEDs rot blinkend
            self.mcu.text("")                                 # Matrix leer
            self.current_label = None                         # Anzeige zurücksetzen
            if new_state == STATE_CALIB2:                     # automatische Kalibrierung starten
                self.calib2_info = {"status": "läuft", "message": "Starte ...", "result": None}  # Fortschritt
                self.calib2 = AutoCalibration(self.cfg, self.mcu, self.vision,  # Thread anlegen
                                              self._calib2_progress, self._calib2_done)  # mit Rückmeldungen
                self.calib2.start()                           # Thread starten
        elif new_state == STATE_HOMING:                       # Referenzfahrt
            self.vision.start_camera()                        # Kamerabild
            self.mcu.led(2, CYAN, 250)                        # beide LEDs türkis blinkend
            self.mcu.text("")                                 # Matrix leer
            self.mcu.radar_servo(None)                        # Radar auf Automatik
            self.mcu.tilt(self.cfg["tilt_level"])             # Kamera waagerecht
            self.mcu.home()                                   # Referenzfahrt starten
        elif new_state == STATE_CALIB_DONE:                   # Kalibrierung fertig
            self.mcu.led(2, GREEN, 0)                         # beide LEDs grün (Dauerlicht)
            self.mcu.text("")                                 # Matrix leer
        elif new_state == STATE_ACTIVE:                       # Erkennung beginnt
            self.vision.start_camera()                        # Kamera an
            self.mcu.led(0, OFF, 0)                           # LED 1 aus (bis etwas erkannt wird)
            self.mcu.led(1, OFF, 0)                           # LED 2 aus
            self.mcu.text("")                                 # Matrix leer
            self.current_label = ""                           # nichts angezeigt
            self.last_detection = time.monotonic()            # Ruhemodus-Zeit neu starten
            self.tracker.reset()                              # Nachführung zurücksetzen
            if old in (STATE_HOMING, STATE_CALIB_DONE) and self.motion_enabled(st):  # nach Referenz/Kalibrierung
                self.mcu.radar_servo(None)                    # Radar auf Automatik
                self.mcu.pan_to(self.cfg["pan_center"])       # Drehachse in Grundstellung
                self.mcu.tilt(self.cfg["tilt_level"])         # Kamera waagerecht
        elif new_state == STATE_SLEEP:                        # Ruhemodus
            self.vision.stop_camera()                         # Kamera aus (Strom sparen)
            self.mcu.led(0, OFF, 0)                           # LED 1 aus
            self.mcu.led(1, ORANGE, 500)                      # LED 2 blinkt orange im Sekundentakt
            self.mcu.text("")                                 # Matrix leer
            self.current_label = ""                           # nichts angezeigt
            self._wake_frames = 0                             # Radar-Weckzähler zurücksetzen
            self._vib_seq = self.mcu.get_vibration()[0]       # alte Vibrationen nicht als Weckgrund zählen
            if st["online"] and st["motors_present"]:         # Motoren vorhanden?
                self.mcu.stop()                               # Drehachse anhalten (Spulen gehen aus)
                self.mcu.tilt(self.cfg["tilt_level"])         # Kamera-Servo waagerecht
                self.mcu.radar_servo(None)                    # Radar-Servo per Automatik ebenfalls waagerecht
            self.vision.publish_placeholder("Ruhemodus - Kamera aus")  # Ersatzbild für die Webseite
        elif new_state == STATE_ERROR:                        # Fehler
            self.error_text = reason                          # Beschreibung merken
            self.vision.start_camera()                        # Kamerabild weiter anzeigen
            self.mcu.led(2, RED, 150)                         # beide LEDs schnell rot blinkend
            self.mcu.text("ERR")                              # Matrix "ERR"
            self.current_label = None                         # Anzeige zurücksetzen

    def _enter_operating_state(self, st):                     # wählt nach dem Start den passenden Zustand
        if not st["motors_present"]:                          # keine Motoren angeschlossen (A0)
            self.log("Keine Servos/Schrittmotor angeschlossen (A0) – nur Erkennung ohne Bewegung")  # Hinweis
            self.set_state(STATE_ACTIVE)                      # direkt erkennen
        elif not self.calibrated():                           # noch nicht kalibriert
            self.set_state(STATE_CALIB)                       # Kalibrierung 1
        else:                                                 # kalibriert
            self.set_state(STATE_HOMING)                      # erst Referenzfahrt (Offset setzen)

    # ================================================================ Zustände ========
    def _run_boot(self, now, st):                             # wartet auf den Sketch
        if st["online"] and st["config_ok"] and not self._config_dirty:  # Sketch da und konfiguriert
            self._motors_prev = bool(st["motors_present"])    # Startzustand von A0 merken
            self._enter_operating_state(st)                   # passenden Zustand wählen
            return                                            # fertig
        time.sleep(0.1)                                       # kurz warten

    def _run_preview(self, now, st):                          # nur Kamerabild (Kalibrierung, Fehler)
        if not self.vision.start_camera():                    # Kamera nicht verfügbar?
            self._placeholder("Kamera nicht verfügbar", now)  # Ersatzbild
            time.sleep(0.2)                                   # kurz warten
            return                                            # fertig
        raw = self.vision.capture()                           # Bild holen (wartet passend zur Bildrate)
        if raw is None:                                       # kein Bild
            time.sleep(0.02)                                  # kurz warten
            return                                            # fertig
        self._count_frame(now)                                # Bildrate messen
        frame = orient(raw, self.flipped)                     # bei Deckenmontage drehen
        info = STATE_TEXT[self.state]                         # Statustext im Bild
        self.vision.publish(self.vision.annotate(frame, info=info))  # an die Webseite

    def _run_calib_done(self, now, st):                       # 5 s grün, dann Erkennung
        self._run_preview(now, st)                            # Kamerabild weiter anzeigen
        if now - self.state_since >= 5.0:                     # 5 Sekunden vorbei?
            self.set_state(STATE_ACTIVE)                      # Objekterkennung starten

    def _run_homing(self, now, st):                           # wartet auf die Referenzfahrt
        self._run_preview(now, st)                            # Kamerabild weiter anzeigen
        if now - self.state_since < 0.6:                      # Status braucht kurz, bis er die Fahrt zeigt
            return                                            # noch warten
        if st["homing_state"] == mcu_mod.HOMING_FAILED:       # fehlgeschlagen
            self.set_state(STATE_ERROR, "Referenzfahrt fehlgeschlagen – Kontaktschalter nicht gefunden")  # Fehler
        elif st["homed"] and st["homing_state"] == mcu_mod.HOMING_DONE:  # erfolgreich
            self.set_state(STATE_ACTIVE, "Referenzfahrt erfolgreich")  # Erkennung starten
        elif now - self.state_since > 90.0:                   # dauert viel zu lange
            self.set_state(STATE_ERROR, "Referenzfahrt: Zeitüberschreitung")  # Fehler

    def _run_sleep(self, now, st):                            # Ruhemodus: nur Weckbedingungen prüfen
        self._placeholder("Ruhemodus - Kamera aus", now)      # Ersatzbild ab und zu erneuern
        if self._wake_reason:                                 # gibt es einen Weckgrund?
            reason, self._wake_reason = self._wake_reason, ""  # übernehmen und zurücksetzen
            self.set_state(STATE_ACTIVE, f"geweckt durch {reason}")  # aufwachen
            return                                            # fertig
        time.sleep(0.1)                                       # wenig Rechenzeit verbrauchen

    def _run_active(self, now, st):                           # Objekterkennung und Nachführung
        if not self.vision.start_camera():                    # Kamera nicht verfügbar?
            self._placeholder("Kamera nicht verfügbar", now)  # Ersatzbild
            time.sleep(0.2)                                   # kurz warten
            return                                            # fertig
        raw = self.vision.capture()                           # Bild holen
        if raw is None:                                       # kein Bild
            time.sleep(0.02)                                  # kurz warten
            return                                            # fertig
        now = time.monotonic()                                # Zeit nach der Aufnahme
        self._count_frame(now)                                # Bildrate messen
        frame = orient(raw, self.flipped)                     # bei Deckenmontage drehen
        h, w = frame.shape[:2]                                # Bildgröße
        cfg = self.cfg                                        # Kurzname

        markers = self.vision.detect_markers(frame)           # 1. Muster suchen (höchste Priorität)
        detections, persons, cars = [], [], []                # Ergebnislisten
        if markers:                                           # Muster gefunden
            self.last_seen["marker"] = now                    # Sichtung merken
        else:                                                 # kein Muster → KI-Objekterkennung
            result = self.vision.detect_objects(frame)        # 2. Personen und Autos suchen
            detections = result or []                         # None (Fehler) wie "nichts" behandeln
            persons = [d for d in detections if d.kind == "person"]  # Personen
            cars = [d for d in detections if d.kind == "car"]  # Autos
            if persons:                                       # Person gefunden
                self.last_seen["person"] = now                # Sichtung merken
            if cars:                                          # Auto gefunden
                self.last_seen["car"] = now                   # Sichtung merken
        if markers or persons or cars:                        # irgendetwas erkannt
            self.last_detection = now                         # Ruhemodus-Zeit neu starten
        self.last_detections = [{"label": d.label, "kind": d.kind, "confidence": round(d.confidence)} for d in detections]  # für Webseite
        if markers:                                           # Muster auch in der Liste zeigen
            self.last_detections.insert(0, {"label": f"marker {markers[0].marker_id}", "kind": "marker", "confidence": 100})  # vorne einfügen

        self._apply_outputs(self._label(now))                 # Matrix-Text und LED 1 setzen

        target, selected_box = None, None                     # Nachführziel
        self.radar_selected = None                            # noch kein Radar-Ziel gewählt
        if markers:                                           # Muster hat Vorrang
            target = markers[0].center                        # Mitte des größten Markers
        elif persons:                                         # sonst die (nächste) Person
            idx = self._choose_person(persons, w, h)          # Person auswählen (Radar hilft bei mehreren)
            selected_box = persons[idx].box                   # deren Rahmen
            x1, y1, x2, y2 = selected_box                     # Ecken
            target = ((x1 + x2) / 2.0, y1 + 0.35 * (y2 - y1))  # Oberkörper statt Bauch anpeilen
        if target is not None and self.motion_enabled(st):    # Ziel vorhanden und Motoren bereit
            raw_target = to_raw_point(target[0], target[1], w, h, self.flipped)  # ins Rohbild umrechnen
            self.tracker.update(raw_target, w, h, st, cfg, self.mcu)  # Kamera nachführen

        remaining = cfg["sleep_timeout_s"] - (now - self.last_detection)  # Zeit bis zum Ruhemodus
        info = f"{self.fps:.1f} fps  Ruhemodus in {max(0, int(remaining))} s"  # Statuszeile im Bild
        img = self.vision.annotate(frame, detections, markers, target, selected_box, info)  # Ergebnisse einzeichnen
        self.vision.publish(img)                              # an die Webseite
        if remaining <= 0:                                    # Zeit abgelaufen
            self.set_state(STATE_SLEEP, f"{int(cfg['sleep_timeout_s'])} s nichts erkannt")  # Ruhemodus

    # ================================================================ Erkennung =======
    def _label(self, now):                                    # was soll angezeigt werden?
        hold = self.cfg["hold_time_s"]                        # Haltezeit gegen Flackern
        for kind in LABEL_PRIORITY:                           # nach Priorität
            if now - self.last_seen[kind] <= hold:            # kürzlich gesehen?
                return kind                                   # diese Art anzeigen
        return ""                                             # nichts

    def _apply_outputs(self, label):                          # setzt Matrix-Text und LED 1
        if label == self.current_label:                       # unverändert?
            return                                            # nichts tun
        self.current_label = label                            # merken
        texts = {"marker": self.cfg["text_marker"], "person": self.cfg["text_person"], "car": self.cfg["text_car"]}  # Texte
        colors = {"marker": WHITE, "person": RED, "car": BLUE}  # LED-1-Farben
        self.mcu.text(texts.get(label, ""))                   # Matrix-Text (leer = aus)
        self.mcu.led(0, colors.get(label, OFF), 0)            # LED 1 Dauerlicht in der Farbe (oder aus)
        if label:                                             # etwas erkannt?
            self.log({"marker": "Muster erkannt", "person": "Person erkannt", "car": "Auto erkannt"}[label])  # Meldung

    def _choose_person(self, persons, w, h):                  # wählt die zu verfolgende Person
        if len(persons) == 1 and not self.radar_targets:      # nur eine Person, kein Radar
            return 0                                          # diese
        boxes = [to_raw_box(p.box, w, h, self.flipped) for p in persons]  # Rahmen im Rohbild (Radar-Koordinaten)
        idx, target = choose_person(boxes, self.radar_targets, w, self.cfg["camera_hfov_deg"])  # Zuordnung
        self.radar_selected = target.index if target is not None else None  # Radar-Ziel für die Anzeige merken
        return max(0, idx)                                    # Index (mindestens 0)

    # ================================================================ Sensoren ========
    def _update_orientation(self, st):                        # erkennt die Deckenmontage
        mode = self.cfg["mount_mode"]                         # eingestellte Montageart
        flipped = self.flipped                                # bisheriger Zustand
        if mode == "normal":                                  # fest normal
            flipped = False                                   # nicht drehen
        elif mode == "ceiling":                               # fest Decke
            flipped = True                                    # drehen
        elif st["online"] and st["mpu_ok"]:                   # automatisch mit dem MPU6050
            axis = self.cfg["mpu_up_axis"]                    # "oben"-Achse (z.B. "z" oder "-y")
            sign = -1 if axis.startswith("-") else 1          # Vorzeichen
            g = sign * st["acc_" + axis[-1]]                  # Erdbeschleunigung entlang "oben" [mg]
            if g < -500:                                      # deutlich negativ → steht auf dem Kopf
                flipped = True                                # Deckenmontage
            elif g > 500:                                     # deutlich positiv → normal
                flipped = False                               # normale Montage
        if flipped != self.flipped:                           # hat sich etwas geändert?
            self.flipped = flipped                            # übernehmen
            self.log("Deckenmontage erkannt – Bild wird gedreht" if flipped else "Normale Montage erkannt")  # Meldung

    def _update_radar(self, now):                             # wertet neue Radar-Daten aus
        raw, t, seq = self.mcu.get_radar()                    # letzte Rohwerte
        if seq != self._radar_seq:                            # neue Meldung?
            self._radar_seq = seq                             # merken
            self.radar_targets = parse_targets(raw, self.cfg["radar_mirror_x"])  # gültige Ziele
            self.radar_time = t                               # Zeitpunkt
            dist = self.cfg["radar_wake_distance_mm"]         # Weckabstand
            speed = self.cfg["radar_wake_min_speed_cms"]      # Weckgeschwindigkeit
            if any(is_wake_target(x, dist, speed) for x in self.radar_targets):  # Ziel erfüllt die Schwelle?
                self._wake_frames += 1                        # mitzählen
            else:                                             # nein
                self._wake_frames = 0                         # zurücksetzen
            if self.state == STATE_SLEEP and self._wake_frames >= self.cfg["radar_wake_frames"]:  # genug hintereinander
                self._wake_reason = "Radar (Lebewesen erfasst)"  # Weckgrund setzen
        if self.radar_targets and now - self.radar_time > 1.5:  # Daten zu alt?
            self.radar_targets = []                           # verwerfen
        if now - self._radar_sent >= 0.1:                     # höchstens 10x pro Sekunde an die Webseite
            self._radar_sent = now                            # Zeitpunkt merken
            sign = -1 if self.flipped else 1                  # bei Deckenmontage links/rechts tauschen
            targets = []                                      # Liste für die Webseite
            for tg in self.radar_targets:                     # jedes Ziel
                d = tg.to_dict()                              # als Wörterbuch
                d["x"] = sign * d["x"]                        # Anzeige aus Sicht des Raumes
                d["angle"] = sign * d["angle"]                # Winkel ebenso
                d["selected"] = (tg.index == self.radar_selected)  # verfolgte Person?
                targets.append(d)                             # hinzufügen
            st = self.mcu.get_status()                        # Status (Radar ok?)
            self.send("radar", {"targets": targets, "ok": bool(st["radar_ok"]),  # an die Webseite
                                "max_range": self.cfg["radar_max_range_mm"],  # Reichweite
                                "hfov": self.cfg["camera_hfov_deg"],  # Kamera-Sichtfeld
                                "wake_distance": self.cfg["radar_wake_distance_mm"]})  # Weckabstand

    def _update_vibration(self):                              # wertet Vibrationsmeldungen aus
        seq, level = self.mcu.get_vibration()                 # Zähler und Stärke
        if seq != self._vib_seq:                              # neue Vibration?
            self._vib_seq = seq                               # merken
            self.log(f"Vibration erkannt ({level} mg)")       # Meldung (LEDs blitzen bereits lila)
            if self.state == STATE_SLEEP:                     # im Ruhemodus?
                self._wake_reason = "Vibration"               # Weckgrund setzen

    def _check_motors(self, st):                              # reagiert auf An-/Abstecken der Motoren
        if self.state == STATE_BOOT or not st["online"]:      # beim Start oder ohne Sketch
            return                                            # nichts tun
        present = bool(st["motors_present"])                  # aktueller Zustand
        if self._motors_prev is None:                         # erster Wert
            self._motors_prev = present                       # merken
            return                                            # fertig
        if present == self._motors_prev:                      # keine Änderung
            return                                            # fertig
        self._motors_prev = present                           # neuen Zustand merken
        if not present:                                       # Motoren entfernt
            self.log("Motoren getrennt (A0 = 0 V) – nur noch Erkennung")  # Meldung
            if self.state not in (STATE_ACTIVE, STATE_SLEEP):  # gerade Kalibrierung/Referenzfahrt?
                self.set_state(STATE_ACTIVE)                  # nur noch erkennen
        else:                                                 # Motoren angeschlossen
            self.log("Motoren angeschlossen (A0 = 3,3 V)")    # Meldung
            if self.state in (STATE_ACTIVE, STATE_SLEEP):     # im Erkennungsbetrieb?
                self.set_state(STATE_HOMING if self.calibrated() else STATE_CALIB)  # Referenzfahrt oder Kalibrierung

    def _process_events(self):                                # Ereignisse aus dem Sketch protokollieren
        names = {                                             # lesbare Namen
            mcu_mod.EVT_HOMING_DONE: "Referenzfahrt fertig",  # 1
            mcu_mod.EVT_HOMING_FAILED: "Referenzfahrt fehlgeschlagen",  # 2
            mcu_mod.EVT_CONFIG_OK: "Konfiguration im Mikrocontroller übernommen",  # 5
            mcu_mod.EVT_SWITCH_HIT: "Kontaktschalter erreicht – Position korrigiert",  # 6
        }
        for code, _value in self.mcu.pop_events():            # alle neuen Ereignisse
            if code in names:                                 # bekanntes Ereignis
                self.log(names[code])                         # protokollieren

    def _sync_config(self, st, now):                          # bringt die Konfiguration in den Sketch
        if not st["online"]:                                  # Sketch nicht erreichbar
            return                                            # später
        need = self._config_dirty or not st["config_ok"]      # geändert oder Sketch neu gestartet
        if need and now - self._config_sent > 1.0:            # höchstens 1x pro Sekunde versuchen
            self._config_sent = now                           # Zeitpunkt merken
            if self.mcu.send_config(self.cfg.mcu_vector()):   # übertragen
                self._config_dirty = False                    # erledigt
                if self.state not in (STATE_BOOT,):           # nach Neustart des Sketches ...
                    self._refresh_outputs()                   # ... LEDs/Text wiederherstellen

    def _refresh_outputs(self):                               # setzt LEDs und Matrix passend zum Zustand neu
        self.mcu.invalidate_cache()                           # Zwischenspeicher leeren
        state = self.state                                    # aktueller Zustand
        if state in CALIB_STATES:                             # nicht kalibriert
            self.mcu.led(2, RED, 500)                         # rot blinkend
        elif state == STATE_HOMING:                           # Referenzfahrt
            self.mcu.led(2, CYAN, 250)                        # türkis blinkend
        elif state == STATE_CALIB_DONE:                       # kalibriert
            self.mcu.led(2, GREEN, 0)                         # grün
        elif state == STATE_SLEEP:                            # Ruhemodus
            self.mcu.led(0, OFF, 0)                           # LED 1 aus
            self.mcu.led(1, ORANGE, 500)                      # LED 2 orange blinkend
        elif state == STATE_ERROR:                            # Fehler
            self.mcu.led(2, RED, 150)                         # schnell rot
            self.mcu.text("ERR")                              # "ERR"
        elif state == STATE_ACTIVE:                           # Erkennung
            self.mcu.led(1, OFF, 0)                           # LED 2 aus
            self.current_label = None                         # Anzeige beim nächsten Bild neu setzen

    # ================================================================ Kalibrierung 2 ==
    def _calib2_progress(self, text):                         # Fortschritt aus dem Kalibrier-Thread
        self.calib2_info["message"] = text                    # Text merken (Webseite zeigt ihn)

    def _calib2_done(self, result):                           # Ergebnis aus dem Kalibrier-Thread
        self.commands.put(("_calib2_result", result))         # an die Hauptschleife übergeben (Thread-sicher)

    def _handle_calib2_result(self, result):                  # Ergebnis in der Hauptschleife verarbeiten
        self.calib2 = None                                    # Thread ist beendet
        self.calib2_info = {"status": "ok" if result.get("ok") else "fehlgeschlagen",  # Status
                            "message": result.get("message", ""), "result": result}  # Meldung und Daten
        if self.state != STATE_CALIB2:                        # inzwischen anderer Zustand (abgebrochen)?
            return                                            # Ergebnis verwerfen
        if result.get("ok"):                                  # automatisch erfolgreich
            pan_dir, tilt_dir = result["pan_dir"], result["tilt_dir"]  # erkannte Richtungen
            self.cfg.update({"pan_frac_per_step": result["pan_frac"], "tilt_frac_per_deg": result["tilt_frac"],  # Messwerte
                             "pan_dir": pan_dir, "tilt_dir": tilt_dir, "calib2_method": "auto",  # Richtungen
                             "calib2_done": True})            # Kalibrierung 2 erledigt
            self.send_config()                                # Webseite aktualisieren
            self.set_state(STATE_CALIB_DONE, result.get("message", ""))  # 5 s grün
        else:                                                 # nicht erfolgreich
            self.set_state(STATE_CALIB2_MANUAL, result.get("message", ""))  # Handeingabe

    def _save_calib2_manual(self, data):                      # speichert die von Hand eingetragenen Richtungen
        pan_dir = 1 if int(data.get("pan_dir", 1)) >= 0 else -1  # Drehrichtung +1/-1
        tilt_dir = 1 if int(data.get("tilt_dir", 1)) >= 0 else -1  # Neigungsrichtung +1/-1
        values = self.cfg.snapshot()                          # aktuelle Werte als Basis
        values.update({"pan_dir": pan_dir, "tilt_dir": tilt_dir})  # eingetragene Richtungen einsetzen
        pan_frac, tilt_frac = manual_fractions(values)        # Umrechnung aus Bildwinkel
        res = (self.calib2_info or {}).get("result") or {}    # Ergebnis der automatischen Messung (falls da)
        if res.get("pan", {}).get("ok") and res.get("pan_dir") == pan_dir:  # Messung passt zur Eingabe?
            pan_frac = res["pan_frac"]                        # genaueren Messwert verwenden
        if res.get("tilt", {}).get("ok") and res.get("tilt_dir") == tilt_dir:  # Messung passt zur Eingabe?
            tilt_frac = res["tilt_frac"]                      # genaueren Messwert verwenden
        self.cfg.update({"pan_frac_per_step": pan_frac, "tilt_frac_per_deg": tilt_frac,  # Werte speichern
                         "pan_dir": pan_dir, "tilt_dir": tilt_dir,  # Richtungen speichern
                         "calib2_method": "manuell", "calib2_done": True})  # Kalibrierung 2 erledigt
        self.send_config()                                    # Webseite aktualisieren
        self.set_state(STATE_CALIB_DONE, "Richtungen von Hand gespeichert")  # 5 s grün

    # ================================================================ Webseiten-Befehle
    def _process_commands(self):                              # führt Befehle der Webseite aus
        while True:                                           # alle wartenden Befehle
            try:                                              # Warteschlange leer?
                name, data = self.commands.get_nowait()       # nächsten Befehl holen
            except queue.Empty:                               # nichts mehr da
                return                                        # fertig
            try:                                              # Fehler im Befehl abfangen
                self._command(name, data or {})               # ausführen
            except Exception as e:                            # Fehler
                logger.exception(f"Befehl {name} fehlgeschlagen: {e}")  # protokollieren
                self.log(f"Befehl {name} fehlgeschlagen: {e}")  # Meldung

    def _command(self, name, data):                           # ein Befehl der Webseite
        st = self.mcu.get_status()                            # aktueller Status
        if name == "_calib2_result":                          # Ergebnis der Kalibrierung 2 (intern)
            self._handle_calib2_result(data)                  # verarbeiten
        elif name == "cal_enter":                             # Kalibrierung öffnen
            if not st["motors_present"]:                      # ohne Motoren nicht möglich
                self.log("Kalibrierung nicht möglich: keine Motoren angeschlossen (A0)")  # Hinweis
            elif self.state not in CALIB_STATES:              # noch nicht in der Kalibrierung
                self.mcu.stop()                               # Bewegungen anhalten
                self.set_state(STATE_CALIB, "über die Webseite")  # Kalibrierung 1
        elif name == "cal_cancel":                            # Kalibrierung verlassen
            if self.state in CALIB_STATES and self.calibrated():  # nur wenn vorher vollständig kalibriert
                self.set_state(STATE_ACTIVE if st["homed"] else STATE_HOMING, "Kalibrierung abgebrochen")  # zurück
            else:                                             # nicht kalibriert
                self.log("Abbrechen nicht möglich: System ist noch nicht kalibriert")  # Hinweis
        elif name == "cal_save1":                             # Kalibrierung 1 speichern
            self._save_calib1(data)                           # prüfen und speichern
        elif name == "cal_start2":                            # "Bereit zur Kamerabewegung einstellen"
            if self.state not in (STATE_CALIB, STATE_CALIB2_MANUAL):  # falscher Zustand
                self.log("Kalibrierung 2 nur aus der Kalibrierung heraus möglich")  # Hinweis
            elif not self.cfg["calib1_done"]:                 # Kalibrierung 1 fehlt
                self.log("Bitte zuerst Kalibrierung 1 speichern")  # Hinweis
            else:                                             # alles in Ordnung
                self.set_state(STATE_CALIB2)                  # automatische Messung starten
        elif name == "cal_save2":                             # Richtungen von Hand speichern
            if self.state in (STATE_CALIB2_MANUAL, STATE_CALIB) and self.cfg["calib1_done"]:  # erlaubt?
                self._save_calib2_manual(data)                # speichern
            else:                                             # nicht erlaubt
                self.log("Speichern nicht möglich: erst Kalibrierung 1 abschließen")  # Hinweis
        elif name == "cal_reset":                             # Kalibrierung komplett zurücksetzen
            self.cfg.update({"calib1_done": False, "calib2_done": False})  # Flags löschen
            self._config_dirty = True                         # Sketch informieren (Grenzen aus)
            self.send_config()                                # Webseite aktualisieren
            if st["motors_present"]:                          # Motoren vorhanden?
                self.set_state(STATE_CALIB, "zurückgesetzt")  # Kalibrierung 1
        elif name == "settings_save":                         # Einstellungen speichern
            values = {k: v for k, v in data.items() if k in SETTINGS_KEYS}  # nur erlaubte Schlüssel
            changed = self.cfg.update(values)                 # prüfen und speichern
            if any(k.startswith("camera_") and k not in ("camera_hfov_deg", "camera_vfov_deg") for k in changed):  # Kamera geändert?
                if self.vision.camera_running():              # läuft sie gerade?
                    self.vision.stop_camera()                 # neu starten (mit neuen Werten)
            self._config_dirty = True                         # Sketch aktualisieren (Schwelle, Helligkeit ...)
            self.current_label = None                         # Texte neu anzeigen
            self.send_config()                                # Webseite aktualisieren
            self.log(f"Einstellungen gespeichert ({len(changed)} geändert)")  # Meldung
        elif name == "wake":                                  # Aufwecken über die Webseite
            if self.state == STATE_SLEEP:                     # nur im Ruhemodus
                self._wake_reason = "Webseite"                # Weckgrund
        elif name == "sleep":                                 # Ruhemodus über die Webseite
            if self.state == STATE_ACTIVE:                    # nur aus der Erkennung
                self.set_state(STATE_SLEEP, "über die Webseite")  # Ruhemodus
        elif name == "retry_homing":                          # Referenzfahrt wiederholen
            if st["motors_present"] and self.calibrated():    # nur kalibriert mit Motoren
                self.set_state(STATE_HOMING, "über die Webseite")  # Referenzfahrt

    def _save_calib1(self, data):                             # prüft und speichert Kalibrierung 1
        values = {k: data[k] for k in CALIB1_KEYS if k in data}  # nur erlaubte Schlüssel
        test = self.cfg.snapshot()                            # aktuelle Werte als Basis
        for k, v in values.items():                           # neue Werte eintragen ...
            test[k] = self.cfg.coerce(k, v)                   # ... geprüft wie beim Speichern
        errors = []                                           # Fehlerliste
        if None in test.values():                             # ungültige Eingabe
            errors.append("ungültige Zahl")                   # Fehler
        else:                                                 # Bereiche prüfen
            if not test["tilt_min"] < test["tilt_level"] < test["tilt_max"]:  # Reihenfolge Servo 1
                errors.append("Servo 1: Minimum < Waagerecht < Maximum")  # Fehler
            if not test["radar_min"] <= test["radar_level"] <= test["radar_max"] or test["radar_min"] >= test["radar_max"]:  # Servo 2
                errors.append("Servo 2: Minimum <= Waagerecht <= Maximum")  # Fehler
            if not test["pan_min"] < test["pan_center"] < test["pan_max"]:  # Drehachse
                errors.append("Drehachse: Minimum < Grundstellung < Maximum")  # Fehler
            if test["servo_us_min"] >= test["servo_us_max"]:  # Impulsgrenzen
                errors.append("Servo-Impulse: Minimum < Maximum")  # Fehler
        if errors:                                            # Fehler gefunden
            self.log("Kalibrierung 1 nicht gespeichert: " + "; ".join(errors))  # Meldung
            self.send("calib_error", {"errors": errors})      # an die Webseite
            return                                            # nicht speichern
        values["calib1_done"] = True                          # Kalibrierung 1 erledigt
        values["calib2_done"] = False                         # Kalibrierung 2 muss (neu) erfolgen
        self.cfg.update(values)                               # speichern
        self._config_dirty = True                             # Sketch aktualisieren
        self.send_config()                                    # Webseite aktualisieren
        self.log("Kalibrierung 1 gespeichert – weiter mit 'Bereit zur Kamerabewegung einstellen'")  # Meldung
        if self.state != STATE_CALIB:                         # sicherstellen, dass wir in Kalibrierung 1 sind
            self.set_state(STATE_CALIB)                       # Zustand setzen

    # ================================================================ Ausgabe =========
    def _count_frame(self, now):                              # misst die Bildrate
        self._fps_count += 1                                  # Bild zählen
        if now - self._fps_time >= 2.0:                       # alle 2 Sekunden auswerten
            self.fps = self._fps_count / (now - self._fps_time)  # Bilder pro Sekunde
            self._fps_count = 0                               # neu zählen
            self._fps_time = now                              # neuer Beginn

    def _placeholder(self, text, now):                        # Ersatzbild höchstens 1x pro Sekunde
        if now - self._placeholder_time >= 1.0:               # Zeit abgelaufen?
            self._placeholder_time = now                      # merken
            self.vision.publish_placeholder(text)             # Ersatzbild ablegen

    def _publish(self, now, st):                              # schickt den Zustand an die Webseite
        if now - self._published < 0.25:                      # höchstens 4x pro Sekunde
            return                                            # noch nicht
        self._published = now                                 # Zeitpunkt merken
        sleep_in = None                                       # Zeit bis Ruhemodus
        if self.state == STATE_ACTIVE:                        # nur im Erkennungsbetrieb sinnvoll
            sleep_in = max(0, int(self.cfg["sleep_timeout_s"] - (now - self.last_detection)))  # Sekunden
        snapshot = {                                          # Inhalt der Statusmeldung
            "state": self.state,                              # Zustand (Schlüssel)
            "state_text": STATE_TEXT[self.state],             # Zustand (Text)
            "error": self.error_text if self.state == STATE_ERROR else "",  # Fehlertext
            "since": int(now - self.state_since),             # Sekunden im Zustand
            "flipped": self.flipped,                          # Deckenmontage
            "label": self.current_label or "",                # angezeigte Erkennung
            "detections": self.last_detections if self.state == STATE_ACTIVE else [],  # Liste
            "motion_enabled": self.motion_enabled(st),        # Nachführung möglich
            "calibrated": self.calibrated(),                  # vollständig kalibriert
            "camera": self.vision.camera_running(),           # Kamera an
            "fps": round(self.fps, 1),                        # Bildrate
            "sleep_in": sleep_in,                             # Sekunden bis Ruhemodus
            "mcu": {                                          # Werte des Sketches
                "online": st["online"],                       # erreichbar
                "motors_present": bool(st["motors_present"]),  # A0
                "homed": bool(st["homed"]),                   # Referenz bekannt
                "homing_state": st["homing_state"],           # Referenzfahrt
                "pan_pos": st["pan_pos"],                     # Position Drehachse
                "pan_moving": bool(st["pan_moving"]),         # fährt
                "tilt": st["tilt"],                           # Winkel Servo 1
                "radar": st["radar"],                         # Winkel Servo 2
                "switch": bool(st["switch"]),                 # Kontaktschalter
                "acc": [st["acc_x"], st["acc_y"], st["acc_z"]],  # Beschleunigung [mg]
                "vib_peak": st["vib_peak"],                   # Erschütterung [mg]
                "mpu_ok": bool(st["mpu_ok"]),                 # MPU6050 da
                "radar_ok": bool(st["radar_ok"]),             # Radar da
            },
            "calib2": {k: v for k, v in self.calib2_info.items()},  # Fortschritt Kalibrierung 2
            "events": list(self.events)[:15],                 # letzte Meldungen
        }
        with self._lock:                                      # exklusiver Zugriff
            self.state_snapshot = snapshot                    # für die REST-Abfrage merken
        self.send("state", snapshot)                          # an die Webseite

    def get_snapshot(self):                                   # letzter Zustand (für /api/state)
        with self._lock:                                      # exklusiver Zugriff
            return dict(self.state_snapshot)                  # Kopie
