# =====================================================================================
#  Watchmen – Schnittstelle zum Mikrocontroller (Sketch) über die Bridge
# =====================================================================================
#  - Befehle an den Sketch:   Bridge.call("name", ...)   (z.B. Servo stellen, LED setzen)
#  - Meldungen vom Sketch:    Bridge.provide("name", funktion)   (Status, Radar, Vibration)
#  Wichtig: In den Meldungs-Funktionen wird NIE Bridge.call() aufgerufen (Verklemmungsgefahr),
#  dort werden nur Daten gespeichert. Ausgewertet wird im Hauptprogramm.
# =====================================================================================

import threading                                              # Sperren für gleichzeitige Zugriffe
import time                                                   # Zeitstempel
from collections import deque                                 # Warteschlange für Ereignisse

from arduino.app_utils import Bridge, Logger                  # Bridge zum Sketch und Protokollierung

logger = Logger("watchmen.mcu")                               # eigener Logger für dieses Modul

EVT_HOMING_DONE = 1                                           # Ereignis: Referenzfahrt fertig
EVT_HOMING_FAILED = 2                                         # Ereignis: Referenzfahrt fehlgeschlagen
EVT_MOTORS_ON = 3                                             # Ereignis: Motoren angeschlossen
EVT_MOTORS_OFF = 4                                            # Ereignis: Motoren entfernt
EVT_CONFIG_OK = 5                                             # Ereignis: Konfiguration übernommen
EVT_SWITCH_HIT = 6                                            # Ereignis: Schalter beim Fahren getroffen

HOMING_IDLE = 0                                               # Referenzfahrt: nichts zu tun
HOMING_SEARCH = 1                                             # Referenzfahrt: sucht Schalter
HOMING_LEAVE = 2                                              # Referenzfahrt: fährt vom Schalter weg
HOMING_DONE = 3                                               # Referenzfahrt: fertig
HOMING_FAILED = -1                                            # Referenzfahrt: fehlgeschlagen

STATUS_FIELDS = [                                             # Reihenfolge der Werte in "mcu_status"
    "motors_present",                                         # 0: Motoren angeschlossen (A0)
    "config_ok",                                              # 1: Sketch hat Konfiguration
    "homed",                                                  # 2: Referenzposition bekannt
    "homing_state",                                           # 3: Zustand der Referenzfahrt
    "pan_pos",                                                # 4: Position Drehachse [Halbschritte]
    "pan_moving",                                             # 5: Drehachse fährt
    "tilt_x10",                                               # 6: Winkel Servo 1 [0,1 Grad]
    "radar_x10",                                              # 7: Winkel Servo 2 [0,1 Grad]
    "switch",                                                 # 8: Kontaktschalter gedrückt
    "acc_x",                                                  # 9: Beschleunigung X [mg]
    "acc_y",                                                  # 10: Beschleunigung Y [mg]
    "acc_z",                                                  # 11: Beschleunigung Z [mg]
    "vib_peak",                                               # 12: stärkste Erschütterung [mg]
    "mpu_ok",                                                 # 13: MPU6050 vorhanden
    "radar_ok",                                               # 14: Radar liefert Daten
]


class Mcu:                                                    # kapselt die Kommunikation mit dem Sketch
    def __init__(self):                                       # Konstruktor
        self._lock = threading.Lock()                         # schützt die gespeicherten Daten
        self._call_lock = threading.Lock()                    # nur ein Bridge-Aufruf gleichzeitig
        self.status = {name: 0 for name in STATUS_FIELDS}     # letzter Status (alles 0 am Anfang)
        self.status_time = 0.0                                # Zeitpunkt des letzten Status
        self.radar_raw = [0] * 9                              # letzte Radar-Rohwerte (x,y,v je Ziel)
        self.radar_time = 0.0                                 # Zeitpunkt der letzten Radar-Meldung
        self.radar_seq = 0                                    # Zähler für neue Radar-Meldungen
        self.vibration_seq = 0                                # Zähler für Vibrationsmeldungen
        self.vibration_level = 0                              # Stärke der letzten Vibration [mg]
        self.events = deque(maxlen=50)                        # Ereignisse aus dem Sketch
        self._led_cache = {}                                  # zuletzt gesendete LED-Befehle
        self._text_cache = None                               # zuletzt gesendeter Matrix-Text
        Bridge.provide("mcu_status", self._on_status)         # Statusmeldungen empfangen
        Bridge.provide("mcu_radar", self._on_radar)           # Radar-Meldungen empfangen
        Bridge.provide("mcu_vibration", self._on_vibration)   # Vibrationsmeldungen empfangen
        Bridge.provide("mcu_event", self._on_event)           # Ereignisse empfangen

    # ------------------------------------------------------------------ Meldungen ----
    def _on_status(self, *values):                            # wird vom Sketch alle 100 ms aufgerufen
        with self._lock:                                      # exklusiver Zugriff
            for name, value in zip(STATUS_FIELDS, values):    # Werte den Namen zuordnen
                self.status[name] = int(value)                # als Ganzzahl speichern
            self.status_time = time.monotonic()               # Zeitpunkt merken

    def _on_radar(self, *values):                             # wird vom Sketch bei neuen Radar-Daten aufgerufen
        with self._lock:                                      # exklusiver Zugriff
            self.radar_raw = [int(v) for v in values[:9]]     # 9 Rohwerte speichern
            self.radar_time = time.monotonic()                # Zeitpunkt merken
            self.radar_seq += 1                               # Zähler erhöhen (= neue Daten)

    def _on_vibration(self, level):                           # wird vom Sketch bei Erschütterung aufgerufen
        with self._lock:                                      # exklusiver Zugriff
            self.vibration_level = int(level)                 # Stärke merken
            self.vibration_seq += 1                           # Zähler erhöhen (= neue Vibration)

    def _on_event(self, code, value):                         # wird vom Sketch bei Ereignissen aufgerufen
        with self._lock:                                      # exklusiver Zugriff
            self.events.append((int(code), int(value)))       # Ereignis in die Warteschlange

    # ------------------------------------------------------------------ Abfragen -----
    def get_status(self):                                     # Kopie des letzten Status
        with self._lock:                                      # exklusiver Zugriff
            st = dict(self.status)                            # Kopie anlegen
            st["age"] = time.monotonic() - self.status_time if self.status_time else 1e9  # Alter in Sekunden
        st["tilt"] = st["tilt_x10"] / 10.0                    # Winkel Servo 1 in Grad
        st["radar"] = st["radar_x10"] / 10.0                  # Winkel Servo 2 in Grad
        st["online"] = st["age"] < 2.0                        # Sketch antwortet (Status jünger als 2 s)
        return st                                             # zurückgeben

    def get_radar(self):                                      # Kopie der letzten Radar-Werte
        with self._lock:                                      # exklusiver Zugriff
            return list(self.radar_raw), self.radar_time, self.radar_seq  # Werte, Zeit, Zähler

    def get_vibration(self):                                  # Zähler und Stärke der Vibrationen
        with self._lock:                                      # exklusiver Zugriff
            return self.vibration_seq, self.vibration_level   # zurückgeben

    def pop_events(self):                                     # holt alle neuen Ereignisse ab
        with self._lock:                                      # exklusiver Zugriff
            items = list(self.events)                         # alle Ereignisse kopieren
            self.events.clear()                               # Warteschlange leeren
        return items                                          # zurückgeben

    # ------------------------------------------------------------------ Befehle ------
    def call(self, name, *args, timeout=2.0):                 # ruft eine Funktion im Sketch auf
        with self._call_lock:                                 # immer nur ein Aufruf gleichzeitig
            try:                                              # Fehler (z.B. Zeitüberschreitung) abfangen
                return Bridge.call(name, *args, timeout=timeout)  # Aufruf über die Bridge
            except Exception as e:                            # irgendein Fehler
                logger.warning(f"Bridge-Aufruf '{name}' fehlgeschlagen: {e}")  # protokollieren
                return None                                   # None = fehlgeschlagen

    def send_config(self, vector):                            # schickt die Konfiguration an den Sketch
        ok = self.call("set_config", list(vector))            # Zahlenliste übertragen
        if ok:                                                # erfolgreich?
            self.invalidate_cache()                           # LEDs/Text danach sicher neu senden
        return bool(ok)                                       # Ergebnis als True/False

    def tilt(self, degrees, raw=False):                       # Servo 1 stellen (Grad)
        return self.call("tilt", int(round(degrees * 10)), 1 if raw else 0)  # in 0,1 Grad senden

    def radar_servo(self, degrees=None):                      # Servo 2 von Hand stellen (None = Automatik)
        value = -1 if degrees is None else int(round(degrees * 10))  # -1 schaltet zurück auf Automatik
        return self.call("radar_servo", value)                # senden

    def pan_to(self, position):                               # Drehachse auf Position fahren
        return self.call("pan_to", int(position))             # senden

    def pan_jog(self, delta):                                 # Drehachse relativ fahren
        return self.call("pan_jog", int(delta))               # senden

    def home(self):                                           # Referenzfahrt starten
        return self.call("home")                              # senden

    def stop(self):                                           # Drehachse anhalten
        return self.call("stop")                              # senden

    def led(self, index, rgb, blink_ms=0, force=False):       # LED setzen (0 = LED 1, 1 = LED 2, 2 = beide)
        key = ("led", index)                                  # Schlüssel für den Zwischenspeicher
        value = (int(rgb), int(blink_ms))                     # gewünschter Zustand
        if index == 2:                                        # beide LEDs: Einzel-Speicher anpassen
            if not force and self._led_cache.get(("led", 0)) == value and self._led_cache.get(("led", 1)) == value:  # schon so?
                return True                                   # nichts zu tun
        elif not force and self._led_cache.get(key) == value:  # einzelne LED schon so?
            return True                                       # nichts zu tun
        ok = self.call("led", int(index), int(rgb), int(blink_ms))  # senden
        if ok:                                                # erfolgreich?
            if index == 2:                                    # beide LEDs
                self._led_cache[("led", 0)] = value           # LED 1 merken
                self._led_cache[("led", 1)] = value           # LED 2 merken
            else:                                             # eine LED
                self._led_cache[key] = value                  # merken
        return ok                                             # Ergebnis

    def flash(self, rgb, count=3):                            # beide LEDs mehrfach aufblitzen lassen
        return self.call("flash", int(rgb), int(count))       # senden

    def text(self, text, force=False):                        # Text auf der LED-Matrix anzeigen
        text = (text or "")[:40]                              # höchstens 40 Zeichen
        if not force and text == self._text_cache:            # schon angezeigt?
            return True                                       # nichts zu tun
        ok = self.call("text", text)                          # senden
        if ok:                                                # erfolgreich?
            self._text_cache = text                           # merken
        return ok                                             # Ergebnis

    def invalidate_cache(self):                               # vergisst die zuletzt gesendeten Ausgaben
        self._led_cache.clear()                               # LED-Speicher leeren
        self._text_cache = None                               # Text-Speicher leeren
