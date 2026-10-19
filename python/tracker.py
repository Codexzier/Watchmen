# =====================================================================================
#  Watchmen – Nachführung der Kamera (Schrittmotor = Drehen, Servo 1 = Neigen)
# =====================================================================================
#  Prinzip (P-Regler): Abweichung des Ziels von der Bildmitte messen und die Motoren so
#  bewegen, dass ein Teil (track_gain) dieser Abweichung ausgeglichen wird.
#  Die Umrechnung "Pixel ↔ Motorbewegung" stammt aus Kalibrierung 2:
#    pan_frac_per_step  = Bildverschiebung je Halbschritt  (Anteil der Bildbreite)
#    tilt_frac_per_deg  = Bildverschiebung je Grad Neigung (Anteil der Bildhöhe)
#  Das Vorzeichen enthält die Richtung. Alle Rechnungen erfolgen im Rohbild (ungedreht).
# =====================================================================================

import time                                                   # Zeitmessung

MAX_PAN_STEP_PER_CMD = 400                                    # höchstens so viele Halbschritte je Befehl
MAX_TILT_DEG_PER_CMD = 8.0                                    # höchstens so viele Grad je Befehl


def manual_fractions(cfg):                                    # Umrechnung aus Handeingabe (Richtung + Bildwinkel)
    steps_per_deg = cfg["stepper_steps_per_rev"] / 360.0      # Halbschritte pro Grad Drehung
    pan_dir = 1 if cfg["pan_dir"] >= 0 else -1                # Richtung als +1/-1
    tilt_dir = 1 if cfg["tilt_dir"] >= 0 else -1              # Richtung als +1/-1
    # Kamera dreht nach rechts → Bildinhalt wandert nach links (negatives Vorzeichen)
    pan_frac = -pan_dir / (cfg["camera_hfov_deg"] * steps_per_deg)  # Bildanteil je Halbschritt
    # Kamera neigt nach unten → Bildinhalt wandert nach oben (negatives Vorzeichen)
    tilt_frac = -tilt_dir / cfg["camera_vfov_deg"]            # Bildanteil je Grad
    return pan_frac, tilt_frac                                # beide Werte zurückgeben


def directions_from_fractions(pan_frac, tilt_frac):           # leitet die Richtungen aus Messwerten ab
    pan_dir = -1 if pan_frac > 0 else 1                       # Inhalt nach links (negativ) = Kamera nach rechts = +1
    tilt_dir = -1 if tilt_frac > 0 else 1                     # Inhalt nach oben (negativ) = Kamera nach unten = +1
    return pan_dir, tilt_dir                                  # beide Richtungen zurückgeben


class Tracker:                                                # berechnet die Fahrbefehle für das Nachführen
    def __init__(self):                                       # Konstruktor
        self._last_cmd = 0.0                                  # Zeitpunkt des letzten Befehls

    def reset(self):                                          # vergisst den letzten Befehlszeitpunkt
        self._last_cmd = 0.0                                  # zurücksetzen

    def update(self, target_raw, width, height, status, cfg, mcu):  # führt die Kamera zum Ziel nach
        """target_raw = (x, y) im Rohbild. Gibt ein Info-Wörterbuch zurück."""
        now = time.monotonic()                                # aktuelle Zeit
        info = {"pan": None, "tilt": None}                    # Ergebnis (neue Ziele oder None)
        if now - self._last_cmd < cfg["track_interval_s"]:    # nicht zu oft befehlen
            return info                                       # nichts tun
        pan_frac = cfg["pan_frac_per_step"]                   # Kalibrierwert Drehen
        tilt_frac = cfg["tilt_frac_per_deg"]                  # Kalibrierwert Neigen
        if pan_frac == 0.0 or tilt_frac == 0.0:               # (noch) keine Messwerte?
            pan_frac, tilt_frac = manual_fractions(cfg)       # aus Richtung und Bildwinkel berechnen
        err_x = (target_raw[0] - width / 2.0) / width         # Abweichung waagerecht (Anteil der Breite)
        err_y = (target_raw[1] - height / 2.0) / height       # Abweichung senkrecht (Anteil der Höhe)
        dead = cfg["track_deadband"] / 2.0                    # Totzone als Anteil der Bildgröße
        gain = cfg["track_gain"]                              # Verstärkung
        if abs(err_x) > dead:                                 # außerhalb der Totzone?
            steps = -gain * err_x / pan_frac                  # nötige Halbschritte (Inhalt soll um -err_x wandern)
            steps = max(-MAX_PAN_STEP_PER_CMD, min(MAX_PAN_STEP_PER_CMD, steps))  # begrenzen
            target = int(round(status["pan_pos"] + steps))    # neue absolute Position
            target = max(cfg["pan_min"], min(cfg["pan_max"], target))  # im erlaubten Bereich halten
            if target != status["pan_pos"]:                   # wirklich eine Bewegung?
                mcu.pan_to(target)                            # Befehl an den Sketch
                info["pan"] = target                          # merken
        if abs(err_y) > dead:                                 # außerhalb der Totzone?
            deg = -gain * err_y / tilt_frac                   # nötige Winkeländerung
            deg = max(-MAX_TILT_DEG_PER_CMD, min(MAX_TILT_DEG_PER_CMD, deg))  # begrenzen
            target = status["tilt"] + deg                     # neuer absoluter Winkel
            target = max(cfg["tilt_min"], min(cfg["tilt_max"], target))  # im erlaubten Bereich halten
            if abs(target - status["tilt"]) >= 0.2:           # Mindeständerung (Servo-Auflösung)
                mcu.tilt(target)                              # Befehl an den Sketch
                info["tilt"] = round(target, 1)               # merken
        if info["pan"] is not None or info["tilt"] is not None:  # wurde etwas befohlen?
            self._last_cmd = now                              # Zeitpunkt merken
        return info                                           # Ergebnis zurückgeben
