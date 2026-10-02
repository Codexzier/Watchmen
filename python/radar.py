# =====================================================================================
#  Watchmen – Auswertung des AI-Thinker RD-03D (Human Radar)
# =====================================================================================
#  Das Radar liefert bis zu 3 Ziele mit X (seitlich), Y (nach vorne) in mm und der
#  Geschwindigkeit in cm/s. Hier werden die Rohwerte geprüft, für das Wecken aus dem
#  Ruhemodus bewertet und den Personen im Kamerabild zugeordnet.
# =====================================================================================

import math                                                   # Winkelfunktionen
from dataclasses import dataclass                             # einfache Datenklassen

SPEED_SENTINELS = (248, 256)                                  # Platzhalterwerte des Radars ("keine Messung")


@dataclass                                                    # Datenklasse für ein Radar-Ziel
class RadarTarget:
    x: int                                                    # seitlicher Abstand [mm] (+ = rechts vom Radar)
    y: int                                                    # Abstand nach vorne [mm]
    speed: int                                                # Geschwindigkeit [cm/s] (Vorzeichen = Richtung)
    index: int                                                # Nummer im Radar-Rahmen (0..2)

    @property
    def distance(self):                                       # direkter Abstand zum Radar [mm]
        return math.hypot(self.x, self.y)                     # Satz des Pythagoras

    @property
    def angle_deg(self):                                      # Winkel zur Blickrichtung [Grad] (+ = rechts)
        return math.degrees(math.atan2(self.x, self.y))       # atan2(seitlich, vorne)

    def to_dict(self):                                        # für die Webseite
        return {                                              # Wörterbuch mit allen Werten
            "x": self.x,                                      # seitlich [mm]
            "y": self.y,                                      # vorne [mm]
            "speed": self.speed,                              # Geschwindigkeit [cm/s]
            "distance": round(self.distance),                 # Abstand [mm]
            "angle": round(self.angle_deg, 1),                # Winkel [Grad]
            "index": self.index,                              # Nummer
        }


def parse_targets(raw, mirror_x=False):                       # macht aus 9 Rohwerten eine Zielliste
    targets = []                                              # Ergebnisliste
    for i in range(3):                                        # 3 mögliche Ziele
        x, y, v = raw[i * 3], raw[i * 3 + 1], raw[i * 3 + 2]  # Rohwerte des Ziels
        if x == 0 and y == 0:                                 # alles 0 = kein Ziel
            continue                                          # überspringen
        if abs(v) in SPEED_SENTINELS:                         # Platzhalter-Geschwindigkeit = Geisterziel
            continue                                          # überspringen
        if y <= 0:                                            # Ziel hinter dem Radar ist unplausibel
            continue                                          # überspringen
        if mirror_x:                                          # Radar spiegelverkehrt eingebaut?
            x = -x                                            # X-Achse umdrehen
        targets.append(RadarTarget(x=x, y=y, speed=v, index=i))  # gültiges Ziel übernehmen
    return targets                                            # Liste zurückgeben


def is_wake_target(target, max_distance_mm, min_speed_cms):   # erfüllt ein Ziel die Weck-Schwelle?
    if target.distance > max_distance_mm:                     # zu weit weg
        return False                                          # nein
    return abs(target.speed) >= min_speed_cms                 # schnell genug (bewegt sich)?


def predict_image_x(target, width, hfov_deg):                 # wo im Kamerabild müsste das Ziel sein?
    half = math.radians(hfov_deg) / 2.0                       # halber Bildwinkel im Bogenmaß
    angle = math.radians(target.angle_deg)                    # Zielwinkel im Bogenmaß
    rel = math.tan(angle) / math.tan(half)                    # -1 (linker Rand) .. +1 (rechter Rand)
    return width / 2.0 + rel * width / 2.0                    # in Bildpixel umrechnen


def choose_person(persons, targets, width, hfov_deg, tolerance=0.25):  # wählt die Person, die am nächsten steht
    """persons: Liste von (x1, y1, x2, y2) im Rohbild. Rückgabe: (Index, Ziel oder None)."""
    if not persons:                                           # keine Personen
        return -1, None                                       # nichts auswählen
    if len(persons) == 1 and not targets:                     # eine Person, kein Radar
        return 0, None                                        # diese nehmen
    best_index, best_target, best_dist = -1, None, None       # bisher beste Zuordnung
    for i, (x1, y1, x2, y2) in enumerate(persons):            # jede Person im Bild
        cx = (x1 + x2) / 2.0                                  # Bildmitte der Person (waagerecht)
        for t in targets:                                     # jedes Radar-Ziel
            px = predict_image_x(t, width, hfov_deg)          # erwartete Bildposition des Ziels
            if abs(px - cx) > tolerance * width + (x2 - x1) / 2.0:  # zu weit auseinander
                continue                                      # passt nicht zusammen
            if best_dist is None or t.distance < best_dist:   # näher als die bisher beste?
                best_index, best_target, best_dist = i, t, t.distance  # merken
    if best_index >= 0:                                       # Zuordnung gefunden
        return best_index, best_target                        # nächste Person zurückgeben
    areas = [(x2 - x1) * (y2 - y1) for (x1, y1, x2, y2) in persons]  # Flächen der Personen
    return areas.index(max(areas)), None                      # Ersatz: größte Person = vermutlich am nächsten
