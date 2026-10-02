# =====================================================================================
#  Watchmen – Kalibrierung 2: Dreh- und Neigungsrichtung automatisch mit der Kamera messen
# =====================================================================================
#  Ablauf:
#   1. Referenzfahrt (falls nötig), dann Grundstellung (Mitte / waagerecht).
#   2. Drehachse um ca. 6° drehen und zurück – jeweils ein Bild aufnehmen.
#      Per Phasenkorrelation wird gemessen, wohin und wie weit der Bildinhalt wandert.
#   3. Dasselbe mit Servo 1 (Neigung).
#   4. Plausibilität prüfen (Hin- und Rückweg entgegengesetzt, genug Bewegung, Güte).
#  Klappt das nicht (z.B. vor einer einfarbigen Wand), trägt man die Richtungen auf der
#  Webseite von Hand ein.
# =====================================================================================

import threading                                              # eigener Thread für den Ablauf
import time                                                   # Wartezeiten

from arduino.app_utils import Logger                          # Protokollierung

from vision import measure_shift                              # Messung der Bildverschiebung
from tracker import directions_from_fractions                 # Richtungen aus Messwerten

logger = Logger("watchmen.calibration")                       # eigener Logger

TEST_ANGLE_DEG = 6.0                                          # Testbewegung in Grad (Drehen und Neigen)
MIN_SHIFT = 0.015                                             # mindestens 1,5 % Bildverschiebung nötig
MIN_RESPONSE = 0.05                                           # Mindestgüte der Phasenkorrelation
SETTLE_S = 0.8                                                # Beruhigungszeit nach jeder Bewegung


class Cancelled(Exception):                                   # wird ausgelöst, wenn abgebrochen wurde
    pass                                                      # keine weiteren Inhalte


class AutoCalibration(threading.Thread):                      # führt Kalibrierung 2 im Hintergrund aus
    def __init__(self, cfg, mcu, vision, on_progress, on_done):  # Konstruktor
        super().__init__(daemon=True, name="Kalibrierung2")   # Hintergrund-Thread mit Namen
        self.cfg = cfg                                        # Konfiguration
        self.mcu = mcu                                        # Sketch-Schnittstelle
        self.vision = vision                                  # Kamera
        self.on_progress = on_progress                        # Rückmeldung "Fortschritt" (Text)
        self.on_done = on_done                                # Rückmeldung "fertig" (Ergebnis)
        self._cancel = threading.Event()                      # Abbruch-Signal

    def cancel(self):                                         # bricht die Kalibrierung ab
        self._cancel.set()                                    # Signal setzen

    def run(self):                                            # Einstiegspunkt des Threads
        try:                                                  # alle Fehler abfangen
            result = self._calibrate()                        # eigentlicher Ablauf
        except Cancelled:                                     # abgebrochen
            result = {"ok": False, "cancelled": True, "message": "Abgebrochen"}  # Ergebnis
        except Exception as e:                                # sonstiger Fehler
            logger.exception(f"Kalibrierung 2 fehlgeschlagen: {e}")  # protokollieren
            result = {"ok": False, "message": f"Fehler: {e}"}  # Ergebnis
        self.on_done(result)                                  # Ergebnis melden

    # ------------------------------------------------------------------ Hilfen -------
    def _sleep(self, seconds):                                # wartet, reagiert aber auf Abbruch
        if self._cancel.wait(seconds):                        # wurde während des Wartens abgebrochen?
            raise Cancelled()                                 # dann abbrechen

    def _wait_idle(self, tilt_target, timeout=20.0):          # wartet, bis alle Motoren stehen
        deadline = time.monotonic() + timeout                 # spätester Zeitpunkt
        while time.monotonic() < deadline:                    # bis zur Zeitgrenze
            st = self.mcu.get_status()                        # aktueller Status
            pan_still = not st["pan_moving"]                  # Drehachse steht?
            tilt_still = abs(st["tilt"] - tilt_target) < 0.3  # Servo 1 am Ziel?
            if st["online"] and pan_still and tilt_still:     # alles ruhig
                self._sleep(SETTLE_S)                         # noch kurz ausschwingen lassen
                return                                        # fertig
            self._sleep(0.1)                                  # kurz warten und erneut prüfen
        raise RuntimeError("Motoren erreichen das Ziel nicht (Zeitüberschreitung)")  # Fehler

    def _grab(self):                                          # holt ein frisches Bild (nach der Bewegung)
        t = time.monotonic()                                  # ab jetzt
        frame = self.vision.wait_frame(newer_than=t, timeout=4.0)  # auf neues Bild warten
        if frame is None:                                     # kein Bild
            raise RuntimeError("Kamera liefert kein Bild")    # Fehler
        return frame                                          # Bild zurückgeben

    # ------------------------------------------------------------------ Ablauf -------
    def _calibrate(self):                                     # eigentlicher Kalibrierablauf
        cfg, mcu = self.cfg, self.mcu                         # Kurznamen
        st = mcu.get_status()                                 # aktueller Status
        if not st["homed"]:                                   # Referenzposition unbekannt?
            self.on_progress("Referenzfahrt zum Kontaktschalter ...")  # Fortschritt melden
            mcu.home()                                        # Referenzfahrt starten
            self._sleep(0.5)                                  # warten, bis der Status die Fahrt zeigt (alte Werte verwerfen)
            deadline = time.monotonic() + 60.0                # höchstens 60 s
            while not mcu.get_status()["homed"]:              # warten bis bekannt
                if mcu.get_status()["homing_state"] == -1:    # fehlgeschlagen?
                    raise RuntimeError("Referenzfahrt fehlgeschlagen (Schalter nicht gefunden)")  # Fehler
                if time.monotonic() > deadline:               # zu lange
                    raise RuntimeError("Referenzfahrt dauert zu lange")  # Fehler
                self._sleep(0.2)                              # kurz warten
        center = (cfg["pan_min"] + cfg["pan_max"]) // 2       # Mitte des Drehbereichs
        level = cfg["tilt_level"]                             # waagerechte Neigung
        self.on_progress("Fahre in Grundstellung ...")        # Fortschritt melden
        mcu.radar_servo(None)                                 # Radar auf Automatik
        mcu.pan_to(center)                                    # Drehachse zur Mitte
        mcu.tilt(level)                                       # Kamera waagerecht
        self._wait_idle(level)                                # warten bis alles steht

        self.on_progress("Messe Drehrichtung ...")            # Fortschritt melden
        pan = self._measure_pan(center, level)                # Drehachse messen
        self.on_progress("Messe Neigungsrichtung ...")        # Fortschritt melden
        tilt = self._measure_tilt(center, level)              # Neigung messen

        result = {"ok": pan["ok"] and tilt["ok"], "pan": pan, "tilt": tilt}  # Gesamtergebnis
        pan_frac = pan["frac"] if pan["ok"] else 0.0          # Messwert Drehen (oder 0)
        tilt_frac = tilt["frac"] if tilt["ok"] else 0.0       # Messwert Neigen (oder 0)
        pan_dir, tilt_dir = directions_from_fractions(pan_frac, tilt_frac)  # Richtungen ableiten
        result["pan_dir"] = pan_dir if pan["ok"] else None    # erkannte Drehrichtung
        result["tilt_dir"] = tilt_dir if tilt["ok"] else None  # erkannte Neigungsrichtung
        result["pan_frac"] = pan_frac                         # Messwert Drehen
        result["tilt_frac"] = tilt_frac                       # Messwert Neigen
        if result["ok"]:                                      # alles erkannt
            result["message"] = "Dreh- und Neigungsrichtung automatisch erkannt"  # Meldung
        else:                                                 # nicht alles erkannt
            result["message"] = "Automatische Erkennung unsicher – bitte Richtungen eintragen"  # Meldung
        return result                                         # Ergebnis zurückgeben

    def _measure_pan(self, center, level):                    # misst die Drehrichtung
        cfg, mcu = self.cfg, self.mcu                         # Kurznamen
        steps_per_deg = cfg["stepper_steps_per_rev"] / 360.0  # Halbschritte pro Grad
        delta = int(round(TEST_ANGLE_DEG * steps_per_deg))    # Testbewegung in Halbschritten
        if center + delta > cfg["pan_max"]:                   # passt nicht nach oben?
            delta = -delta                                    # dann in die andere Richtung
        f0 = self._grab()                                     # Bild in Grundstellung
        mcu.pan_to(center + delta)                            # drehen
        self._wait_idle(level)                                # warten
        f1 = self._grab()                                     # Bild nach der Drehung
        mcu.pan_to(center)                                    # zurückdrehen
        self._wait_idle(level)                                # warten
        f2 = self._grab()                                     # Bild nach dem Rückweg
        dx1, dy1, r1 = measure_shift(f0, f1)                  # Verschiebung Hinweg
        dx2, dy2, r2 = measure_shift(f1, f2)                  # Verschiebung Rückweg
        frac = (dx1 - dx2) / (2.0 * delta)                    # Mittelwert: Bildanteil je Halbschritt
        expected = 1.0 / (cfg["camera_hfov_deg"] * steps_per_deg)  # erwarteter Betrag laut Bildwinkel
        checks = {                                            # einzelne Prüfungen
            "bewegung": abs(dx1) >= MIN_SHIFT and abs(dx2) >= MIN_SHIFT,  # genug Verschiebung
            "gegenlaeufig": dx1 * dx2 < 0,                    # Hin- und Rückweg entgegengesetzt
            "guete": r1 >= MIN_RESPONSE and r2 >= MIN_RESPONSE,  # Messung zuverlässig
            "waagerecht": abs(dx1) > abs(dy1) and abs(dx2) > abs(dy2),  # Bewegung überwiegend seitlich
            "plausibel": expected / 3.0 <= abs(frac) <= expected * 3.0,  # Größenordnung passt
        }
        logger.info(f"Drehen: dx1={dx1:.4f} dx2={dx2:.4f} r={r1:.2f}/{r2:.2f} frac={frac:.6f} {checks}")  # Protokoll
        return {"ok": all(checks.values()), "frac": frac, "checks": checks,  # Ergebnis
                "shift": [round(dx1, 4), round(dx2, 4)], "response": [round(r1, 3), round(r2, 3)]}  # Messwerte

    def _measure_tilt(self, center, level):                   # misst die Neigungsrichtung
        cfg, mcu = self.cfg, self.mcu                         # Kurznamen
        delta = TEST_ANGLE_DEG                                # Testbewegung in Grad
        if level + delta > cfg["tilt_max"]:                   # passt nicht nach oben?
            delta = -delta                                    # dann in die andere Richtung
        f0 = self._grab()                                     # Bild in Grundstellung
        mcu.tilt(level + delta)                               # neigen
        self._wait_idle(level + delta)                        # warten
        f1 = self._grab()                                     # Bild nach dem Neigen
        mcu.tilt(level)                                       # zurück
        self._wait_idle(level)                                # warten
        f2 = self._grab()                                     # Bild nach dem Rückweg
        dx1, dy1, r1 = measure_shift(f0, f1)                  # Verschiebung Hinweg
        dx2, dy2, r2 = measure_shift(f1, f2)                  # Verschiebung Rückweg
        frac = (dy1 - dy2) / (2.0 * delta)                    # Mittelwert: Bildanteil je Grad
        expected = 1.0 / cfg["camera_vfov_deg"]               # erwarteter Betrag laut Bildwinkel
        checks = {                                            # einzelne Prüfungen
            "bewegung": abs(dy1) >= MIN_SHIFT and abs(dy2) >= MIN_SHIFT,  # genug Verschiebung
            "gegenlaeufig": dy1 * dy2 < 0,                    # Hin- und Rückweg entgegengesetzt
            "guete": r1 >= MIN_RESPONSE and r2 >= MIN_RESPONSE,  # Messung zuverlässig
            "senkrecht": abs(dy1) > abs(dx1) and abs(dy2) > abs(dx2),  # Bewegung überwiegend senkrecht
            "plausibel": expected / 3.0 <= abs(frac) <= expected * 3.0,  # Größenordnung passt
        }
        logger.info(f"Neigen: dy1={dy1:.4f} dy2={dy2:.4f} r={r1:.2f}/{r2:.2f} frac={frac:.5f} {checks}")  # Protokoll
        return {"ok": all(checks.values()), "frac": frac, "checks": checks,  # Ergebnis
                "shift": [round(dy1, 4), round(dy2, 4)], "response": [round(r1, 3), round(r2, 3)]}  # Messwerte
