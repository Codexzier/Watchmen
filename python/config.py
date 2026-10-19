# =====================================================================================
#  Watchmen – Konfiguration (Standardwerte, Laden, Speichern)
# =====================================================================================
#  Alle Einstellungen (Kalibrierung, Schwellwerte, Texte ...) liegen in einer JSON-Datei.
#  Auf dem UNO Q ist das /app/data/watchmen_config.json (bleibt beim Neustart erhalten).
# =====================================================================================

import copy                                                   # tiefe Kopien von Wörterbüchern
import json                                                   # JSON lesen und schreiben
import os                                                     # Pfade und Dateioperationen
import threading                                              # Sperre gegen gleichzeitige Zugriffe

if os.path.isdir("/app"):                                     # läuft die App auf dem UNO Q (Container)?
    DATA_DIR = "/app/data"                                    # dann dauerhafter Datenordner der App
else:                                                         # sonst (z.B. Test am PC)
    DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data")  # Ordner "data" im Projekt
CONFIG_PATH = os.path.join(DATA_DIR, "watchmen_config.json")  # vollständiger Pfad der Konfigurationsdatei

DEFAULTS = {                                                  # Standardwerte aller Einstellungen
    # --- Kalibrierungsstatus ---
    "calib1_done": False,                                     # Kalibrierung 1 (Bereiche) abgeschlossen?
    "calib2_done": False,                                     # Kalibrierung 2 (Richtungen) abgeschlossen?
    # --- Servo 1: Neigung der Plattform/Kamera (Grad) ---
    "tilt_min": 45.0,                                         # kleinster erlaubter Winkel
    "tilt_max": 135.0,                                        # größter erlaubter Winkel
    "tilt_level": 90.0,                                       # Winkel, bei dem die Kamera waagerecht ist
    # --- Servo 2: Neigung des Radars (Grad) ---
    "radar_min": 45.0,                                        # kleinster erlaubter Winkel
    "radar_max": 135.0,                                       # größter erlaubter Winkel
    "radar_level": 90.0,                                      # Winkel, bei dem das Radar waagerecht ist
    "radar_comp_factor": -1.0,                                # Radar = Level + Faktor * (Neigung - Level)
    "radar_comp_enabled": True,                               # Radar automatisch waagerecht halten
    # --- Schrittmotor / Drehachse (Halbschritte, 0 = Kontaktschalter) ---
    "pan_min": 20,                                            # kleinste erlaubte Position
    "pan_max": 3000,                                          # größte erlaubte Position
    "pan_center": 1500,                                       # Grundstellung (Blick nach vorne)
    "stepper_invert": False,                                  # Drehrichtung umkehren (Schalter auf anderer Seite)
    "stepper_steps_per_rev": 4096,                            # Halbschritte pro Umdrehung (28BYJ-48)
    "stepper_max_speed": 500,                                 # Höchstgeschwindigkeit [Halbschritte/s]
    "homing_max_steps": 5000,                                 # maximaler Suchweg der Referenzfahrt
    # --- Servos allgemein ---
    "servo_speed_dps": 90,                                    # Höchstgeschwindigkeit der Servos [Grad/s]
    "servo_us_min": 500,                                      # Impulslänge bei 0 Grad [µs]
    "servo_us_max": 2500,                                     # Impulslänge bei 180 Grad [µs]
    # --- Ergebnis Kalibrierung 2 ---
    "pan_frac_per_step": 0.0,                                 # Bildverschiebung je Halbschritt (Anteil der Bildbreite)
    "tilt_frac_per_deg": 0.0,                                 # Bildverschiebung je Grad Neigung (Anteil der Bildhöhe)
    "pan_dir": 1,                                             # +1: positive Schritte drehen die Kamera nach rechts
    "tilt_dir": 1,                                            # +1: größerer Servowinkel neigt die Kamera nach unten
    "calib2_method": "",                                      # "auto" oder "manuell"
    # --- Kamera ---
    "camera_source": "csi:0",                                 # IMX219 am Media Carrier (leer = automatisch)
    "camera_width": 640,                                      # Bildbreite in Pixeln
    "camera_height": 480,                                     # Bildhöhe in Pixeln
    "camera_fps": 15,                                         # Bilder pro Sekunde
    "camera_hfov_deg": 62.2,                                  # horizontaler Bildwinkel IMX219 (Standardobjektiv)
    "camera_vfov_deg": 48.8,                                  # vertikaler Bildwinkel IMX219
    # --- Montage ---
    "mount_mode": "auto",                                     # "auto" (MPU6050), "normal" oder "ceiling" (Decke)
    "mpu_up_axis": "z",                                       # welche MPU-Achse zeigt bei normaler Montage nach oben
    # --- Erkennung ---
    "detection_confidence": 0.5,                              # Mindest-Sicherheit der Objekterkennung (0..1)
    "person_labels": "person",                                # Klassen, die als Person gelten (Komma-getrennt)
    "car_labels": "car",                                      # Klassen, die als Auto gelten (Komma-getrennt)
    "marker_dictionary": "DICT_4X4_50",                       # ArUco-Wörterbuch für das Muster
    "marker_id": -1,                                          # gesuchte Marker-ID (-1 = jede)
    "hold_time_s": 1.0,                                       # so lange bleibt eine Anzeige nach dem letzten Treffer
    # --- Nachführung (Tracking) ---
    "track_gain": 0.5,                                        # Verstärkung (0..1): Anteil der Abweichung pro Schritt
    "track_deadband": 0.06,                                   # Totzone (Anteil der halben Bildbreite)
    "track_interval_s": 0.15,                                 # Mindestabstand zwischen zwei Fahrbefehlen
    # --- Ruhemodus ---
    "sleep_timeout_s": 60.0,                                  # ohne Erkennung so lange bis Ruhemodus
    "radar_wake_distance_mm": 3000,                           # Radar weckt bei Ziel näher als dieser Abstand
    "radar_wake_min_speed_cms": 1,                            # ... und mindestens dieser Geschwindigkeit
    "radar_wake_frames": 3,                                   # ... in so vielen Radar-Meldungen hintereinander
    "vibration_threshold_mg": 150,                            # Vibrationsschwelle [mg]
    # --- Radar ---
    "radar_mirror_x": False,                                  # X-Achse des Radars spiegeln (Einbaulage)
    "radar_max_range_mm": 8000,                               # Reichweite für die Radar-Anzeige
    # --- Ausgaben ---
    "text_car": "CAR",                                        # Matrix-Text bei Auto
    "text_person": "HUMEN",                                   # Matrix-Text bei Person
    "text_marker": "MARK",                                    # Matrix-Text beim Muster
    "led_brightness": 64,                                     # Helligkeit der WS2812B (0..255)
}

RANGES = {                                                    # erlaubte Wertebereiche (min, max) für Zahlen
    "tilt_min": (0, 180), "tilt_max": (0, 180), "tilt_level": (0, 180),        # Servo 1 in Grad
    "radar_min": (0, 180), "radar_max": (0, 180), "radar_level": (0, 180),     # Servo 2 in Grad
    "radar_comp_factor": (-3.0, 3.0),                         # Ausgleichsfaktor
    "pan_min": (0, 20000), "pan_max": (0, 20000), "pan_center": (0, 20000),    # Drehachse
    "stepper_steps_per_rev": (100, 20000),                    # Schritte pro Umdrehung
    "stepper_max_speed": (150, 1000),                         # Schrittgeschwindigkeit
    "homing_max_steps": (100, 20000),                         # Suchweg
    "servo_speed_dps": (10, 360),                             # Servogeschwindigkeit
    "servo_us_min": (300, 1500), "servo_us_max": (1500, 2800),  # Impulsgrenzen
    "camera_width": (160, 1920), "camera_height": (120, 1080),  # Bildgröße
    "camera_fps": (1, 30),                                    # Bildrate
    "camera_hfov_deg": (10, 180), "camera_vfov_deg": (10, 180),  # Bildwinkel
    "detection_confidence": (0.05, 0.99),                     # Erkennungsschwelle
    "marker_id": (-1, 999),                                   # Marker-ID
    "hold_time_s": (0.0, 10.0),                               # Haltezeit
    "track_gain": (0.05, 1.0),                                # Verstärkung
    "track_deadband": (0.0, 0.5),                             # Totzone
    "track_interval_s": (0.05, 2.0),                          # Befehlsabstand
    "sleep_timeout_s": (5, 3600),                             # Zeit bis Ruhemodus
    "radar_wake_distance_mm": (100, 8000),                    # Weckabstand
    "radar_wake_min_speed_cms": (0, 500),                     # Weckgeschwindigkeit
    "radar_wake_frames": (1, 50),                             # Wecktakte
    "vibration_threshold_mg": (10, 2000),                     # Vibrationsschwelle
    "radar_max_range_mm": (1000, 10000),                      # Anzeige-Reichweite
    "led_brightness": (0, 255),                               # LED-Helligkeit
    "pan_dir": (-1, 1), "tilt_dir": (-1, 1),                  # Richtungen
}

CHOICES = {                                                   # erlaubte Werte für Auswahlfelder
    "mount_mode": ("auto", "normal", "ceiling"),              # Montagearten
    "mpu_up_axis": ("x", "y", "z", "-x", "-y", "-z"),         # mögliche "oben"-Achsen
    "calib2_method": ("", "auto", "manuell"),                 # Kalibrierverfahren
}


def _to_bool(value):                                          # wandelt Eingaben in True/False um
    if isinstance(value, str):                                # Text (z.B. aus der Webseite)
        return value.strip().lower() in ("1", "true", "ja", "yes", "on")  # typische "wahr"-Texte
    return bool(value)                                        # sonst normale Umwandlung


class Config:                                                 # verwaltet die Einstellungen
    def __init__(self, path=CONFIG_PATH):                     # Konstruktor
        self._path = path                                     # Speicherort merken
        self._lock = threading.RLock()                        # Sperre (auch mehrfach im selben Thread nutzbar)
        self._data = copy.deepcopy(DEFAULTS)                  # mit Standardwerten beginnen
        self.load()                                           # gespeicherte Werte laden (falls vorhanden)

    def load(self):                                           # lädt die Datei
        with self._lock:                                      # exklusiver Zugriff
            try:                                              # Fehler beim Lesen abfangen
                with open(self._path, "r", encoding="utf-8") as f:  # Datei öffnen
                    stored = json.load(f)                     # JSON einlesen
            except FileNotFoundError:                         # noch keine Datei vorhanden
                return                                        # Standardwerte behalten
            except (OSError, ValueError):                     # Datei beschädigt oder unlesbar
                return                                        # Standardwerte behalten
            self.update(stored, save=False)                   # bekannte Werte übernehmen (geprüft)

    def save(self):                                           # speichert die Datei (atomar)
        with self._lock:                                      # exklusiver Zugriff
            os.makedirs(os.path.dirname(self._path), exist_ok=True)  # Ordner anlegen, falls nötig
            tmp = self._path + ".tmp"                         # erst in eine temporäre Datei schreiben
            with open(tmp, "w", encoding="utf-8") as f:       # temporäre Datei öffnen
                json.dump(self._data, f, indent=2, ensure_ascii=False)  # lesbar formatiert schreiben
            os.replace(tmp, self._path)                       # dann in einem Schritt ersetzen (kein Halb-Zustand)

    def get(self, key):                                       # liest einen Wert
        with self._lock:                                      # exklusiver Zugriff
            return self._data[key]                            # Wert zurückgeben

    def __getitem__(self, key):                               # erlaubt cfg["schluessel"]
        return self.get(key)                                  # wie get()

    def snapshot(self):                                       # Kopie aller Werte (z.B. für die Webseite)
        with self._lock:                                      # exklusiver Zugriff
            return copy.deepcopy(self._data)                  # tiefe Kopie zurückgeben

    def update(self, values, save=True):                      # übernimmt mehrere Werte (mit Prüfung)
        changed = []                                          # Liste der geänderten Schlüssel
        with self._lock:                                      # exklusiver Zugriff
            for key, value in dict(values).items():           # jeden übergebenen Wert prüfen
                if key not in DEFAULTS:                       # unbekannter Schlüssel
                    continue                                  # ignorieren
                new = self.coerce(key, value)                 # in den richtigen Typ umwandeln
                if new is None:                               # Umwandlung nicht möglich
                    continue                                  # ignorieren
                if self._data[key] != new:                    # nur echte Änderungen
                    self._data[key] = new                     # neuen Wert speichern
                    changed.append(key)                       # Änderung merken
            if save and changed:                              # gab es Änderungen und soll gespeichert werden?
                self.save()                                   # Datei schreiben
        return changed                                        # geänderte Schlüssel zurückgeben

    def coerce(self, key, value):                             # wandelt einen Wert passend zum Standardtyp um (None = ungültig)
        default = DEFAULTS[key]                               # Standardwert bestimmt den Typ
        try:                                                  # Umwandlungsfehler abfangen
            if isinstance(default, bool):                     # Wahrheitswert
                return _to_bool(value)                        # umwandeln
            if isinstance(default, int):                      # Ganzzahl
                new = int(round(float(value)))                # über float runden (auch "12.0" geht)
            elif isinstance(default, float):                  # Kommazahl
                new = float(value)                            # umwandeln
            else:                                             # Text
                new = str(value).strip()[:64]                 # höchstens 64 Zeichen
                if key in CHOICES and new not in CHOICES[key]:  # nur erlaubte Auswahlwerte
                    return None                               # sonst ablehnen
                return new                                    # Text zurückgeben
        except (TypeError, ValueError):                       # Umwandlung fehlgeschlagen
            return None                                       # ablehnen
        if key in RANGES:                                     # gibt es einen erlaubten Bereich?
            lo, hi = RANGES[key]                              # Grenzen holen
            new = min(max(new, lo), hi)                       # in den Bereich begrenzen
            if isinstance(default, int):                      # Ganzzahl bleibt Ganzzahl
                new = int(new)                                # umwandeln
        return new                                            # geprüften Wert zurückgeben

    def labels(self, key):                                    # liefert eine Klassenliste (z.B. "person, man")
        return [s.strip().lower() for s in self.get(key).split(",") if s.strip()]  # zerlegen und säubern

    def mcu_vector(self):                                     # Zahlenliste für rpcSetConfig() im Sketch
        with self._lock:                                      # exklusiver Zugriff
            return mcu_vector_from(self._data)                # aus den aktuellen Werten berechnen


def mcu_vector_from(d):                                       # Zahlenliste aus einem Wörterbuch mit Einstellungen
    return [                                                  # Reihenfolge muss zum Sketch passen!
        int(round(d["tilt_min"] * 10)),                       # 0: Servo 1 Minimum [0,1 Grad]
        int(round(d["tilt_max"] * 10)),                       # 1: Servo 1 Maximum
        int(round(d["tilt_level"] * 10)),                     # 2: Servo 1 waagerecht
        int(round(d["radar_min"] * 10)),                      # 3: Servo 2 Minimum
        int(round(d["radar_max"] * 10)),                      # 4: Servo 2 Maximum
        int(round(d["radar_level"] * 10)),                    # 5: Servo 2 waagerecht
        int(round(d["radar_comp_factor"] * 100)),             # 6: Ausgleichsfaktor [x100]
        1 if d["radar_comp_enabled"] else 0,                  # 7: Ausgleich ein/aus
        int(d["pan_min"]),                                    # 8: Drehachse Minimum
        int(d["pan_max"]),                                    # 9: Drehachse Maximum
        1 if d["stepper_invert"] else 0,                      # 10: Drehrichtung umkehren
        int(d["stepper_max_speed"]),                          # 11: Schrittgeschwindigkeit
        int(d["servo_speed_dps"]),                            # 12: Servogeschwindigkeit
        int(d["vibration_threshold_mg"]),                     # 13: Vibrationsschwelle
        int(d["servo_us_min"]),                               # 14: Impuls 0 Grad
        int(d["servo_us_max"]),                               # 15: Impuls 180 Grad
        1 if d["calib1_done"] else 0,                         # 16: Grenzen aktiv
        int(d["homing_max_steps"]),                           # 17: Suchweg Referenzfahrt
        int(d["led_brightness"]),                             # 18: LED-Helligkeit
        1,                                                    # 19: Versionsnummer der Liste
    ]
