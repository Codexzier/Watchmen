# =====================================================================================
#  Watchmen – Kamera, Muster-Erkennung (ArUco), Objekterkennung und Bildausgabe
# =====================================================================================
#  - Kamera: IMX219 am UNO Media Carrier (CSI), wird im Ruhemodus abgeschaltet.
#  - Muster: ArUco-Marker (markante Schwarz/Weiß-Quadrate, sehr schnell und robust).
#  - Objekte: KI-Modell des "object_detection"-Bricks (u.a. "person" und "car").
#  - Deckenmontage: Das Bild wird um 180° gedreht. Für die Motorsteuerung werden die
#    Koordinaten wieder ins ungedrehte Rohbild umgerechnet (dort wurde kalibriert).
# =====================================================================================

import threading                                              # Sperren und Bedingungsvariablen
import time                                                   # Zeitmessung
from dataclasses import dataclass                             # einfache Datenklassen

import cv2                                                    # OpenCV: Bildverarbeitung
import numpy as np                                            # Zahlen-Arrays (Bilder)

from arduino.app_utils import Logger                          # Protokollierung

logger = Logger("watchmen.vision")                            # eigener Logger für dieses Modul


@dataclass                                                    # Ergebnis der Objekterkennung
class Detection:
    label: str                                                # Klassenname des Modells (z.B. "person")
    kind: str                                                 # "person", "car" oder "other"
    confidence: float                                         # Sicherheit in Prozent (0..100)
    box: tuple                                                # Rahmen (x1, y1, x2, y2) im angezeigten Bild


@dataclass                                                    # Ergebnis der Marker-Erkennung
class Marker:
    marker_id: int                                            # Nummer des ArUco-Markers
    center: tuple                                             # Mittelpunkt (x, y) im angezeigten Bild
    corners: np.ndarray                                       # 4 Eckpunkte
    area: float                                               # Fläche in Pixeln (größer = näher)


def orient(frame, flipped):                                   # dreht das Bild bei Deckenmontage
    return cv2.rotate(frame, cv2.ROTATE_180) if flipped else frame  # 180° drehen oder unverändert


def to_raw_point(x, y, width, height, flipped):               # Punkt im angezeigten Bild → Rohbild
    return (width - x, height - y) if flipped else (x, y)     # bei 180°-Drehung spiegeln


def to_raw_box(box, width, height, flipped):                  # Rahmen im angezeigten Bild → Rohbild
    x1, y1, x2, y2 = box                                      # Ecken auspacken
    if not flipped:                                           # keine Drehung
        return box                                            # unverändert
    return (width - x2, height - y2, width - x1, height - y1)  # gespiegelte Ecken (wieder links/oben zuerst)


def measure_shift(img_a, img_b):                              # misst die Bildverschiebung zwischen zwei Bildern
    """Gibt (dx, dy, Güte) zurück. dx/dy als Anteil der Bildbreite/-höhe (Inhalt wandert von a nach b)."""
    h, w = img_a.shape[:2]                                    # Bildgröße
    scale = 320.0 / w                                         # auf 320 Pixel Breite verkleinern (schneller)
    size = (320, max(16, int(round(h * scale))))              # Zielgröße
    a = cv2.resize(cv2.cvtColor(img_a, cv2.COLOR_BGR2GRAY), size).astype(np.float32)  # Graubild A
    b = cv2.resize(cv2.cvtColor(img_b, cv2.COLOR_BGR2GRAY), size).astype(np.float32)  # Graubild B
    a = cv2.GaussianBlur(a, (0, 0), 1.5)                      # leicht glätten (Rauschen weg)
    b = cv2.GaussianBlur(b, (0, 0), 1.5)                      # leicht glätten
    window = cv2.createHanningWindow(size, cv2.CV_32F)        # Fenster gegen Randeffekte
    (dx, dy), response = cv2.phaseCorrelate(a, b, window)     # Phasenkorrelation: Verschiebung + Güte
    return dx / size[0], dy / size[1], float(response)        # in Bildanteile umrechnen


def marker_png(dictionary_name, marker_id, size=600):         # erzeugt einen druckbaren Marker als PNG
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))  # Wörterbuch laden
    img = cv2.aruco.generateImageMarker(dictionary, int(marker_id), size)  # Marker zeichnen
    border = size // 6                                        # weißer Rand (wichtig für die Erkennung)
    img = cv2.copyMakeBorder(img, border, border, border, border, cv2.BORDER_CONSTANT, value=255)  # Rand anfügen
    cv2.putText(img, f"{dictionary_name}  ID {marker_id}", (border, img.shape[0] - border // 3),  # Beschriftung
                cv2.FONT_HERSHEY_SIMPLEX, size / 900.0, 0, 2, cv2.LINE_AA)  # Schrift, Größe, Farbe schwarz
    ok, png = cv2.imencode(".png", img)                       # als PNG kodieren
    return png.tobytes() if ok else b""                       # Bytes zurückgeben


class Vision:                                                 # Kamera und Bildauswertung
    def __init__(self, cfg, detector_factory):                # Konstruktor
        self.cfg = cfg                                        # Konfiguration
        self._detector_factory = detector_factory             # Funktion, die das KI-Modell anlegt
        self._detector = None                                 # KI-Modell (wird bei Bedarf angelegt)
        self._detector_retry = 0.0                            # Zeitpunkt für neuen Versuch
        self._camera = None                                   # Kamera-Objekt
        self._camera_retry = 0.0                              # Zeitpunkt für neuen Startversuch
        self._last_good_frame = 0.0                           # Zeitpunkt des letzten gültigen Bildes
        self._frame_cond = threading.Condition()              # meldet "neues Bild da"
        self._frame_raw = None                                # letztes Rohbild (ungedreht)
        self._frame_time = 0.0                                # Aufnahmezeitpunkt des letzten Bildes
        self._jpeg_lock = threading.Lock()                    # schützt das JPEG für die Webseite
        self._jpeg = None                                     # letztes JPEG für die Webseite
        self._jpeg_seq = 0                                    # Zähler der JPEGs
        self._aruco = None                                    # ArUco-Detektor
        self._aruco_name = None                               # Name des verwendeten Wörterbuchs
        self._detect_warn_time = 0.0                          # Zeitpunkt der letzten Warnung

    # ------------------------------------------------------------------ Kamera -------
    def camera_running(self):                                 # läuft die Kamera?
        return self._camera is not None                       # ja, wenn ein Objekt existiert

    def start_camera(self):                                   # startet die Kamera (falls nicht aktiv)
        if self._camera is not None:                          # läuft schon
            return True                                       # fertig
        if time.monotonic() < self._camera_retry:             # letzter Versuch ist zu kurz her
            return False                                      # später erneut
        from arduino.app_peripherals.camera import Camera     # erst hier importieren (nur bei Bedarf)
        size = (self.cfg["camera_width"], self.cfg["camera_height"])  # gewünschte Auflösung
        fps = self.cfg["camera_fps"]                          # gewünschte Bildrate
        sources = list(dict.fromkeys([self.cfg["camera_source"] or None, None]))  # erst konfigurierte Quelle, dann automatisch (ohne Doppelte)
        for source in sources:                                # Quellen nacheinander probieren
            try:                                              # Startfehler abfangen
                cam = Camera(source, resolution=size, fps=fps, auto_reconnect=False)  # Kamera anlegen
                cam.start()                                   # Kamera starten
                self._camera = cam                            # merken
                self._last_good_frame = time.monotonic()      # Zeitmessung für Aussetzer starten
                logger.info(f"Kamera gestartet: {source or 'automatisch'} {size} @ {fps} fps")  # Info
                return True                                   # erfolgreich
            except Exception as e:                            # Start fehlgeschlagen
                logger.warning(f"Kamera '{source}' startet nicht: {e}")  # Warnung
        self._camera_retry = time.monotonic() + 10.0          # in 10 s erneut versuchen
        return False                                          # nicht erfolgreich

    def stop_camera(self):                                    # schaltet die Kamera ab (Strom sparen)
        cam, self._camera = self._camera, None                # Objekt entnehmen
        if cam is not None:                                   # gab es eine Kamera?
            try:                                              # Fehler beim Stoppen abfangen
                cam.stop()                                    # stoppen
                logger.info("Kamera gestoppt")                # Info
            except Exception as e:                            # Fehler
                logger.warning(f"Kamera stoppen fehlgeschlagen: {e}")  # Warnung
        with self._frame_cond:                                # exklusiver Zugriff
            self._frame_raw = None                            # altes Bild verwerfen

    def capture(self):                                        # holt ein neues Rohbild (blockiert gemäß fps)
        cam = self._camera                                    # aktuelles Kamera-Objekt
        if cam is None:                                       # keine Kamera
            return None                                       # kein Bild
        try:                                                  # Lesefehler abfangen
            frame = cam.capture()                             # Bild holen (BGR, numpy)
        except Exception as e:                                # Fehler beim Lesen
            logger.warning(f"Kamerafehler: {e}")              # Warnung
            frame = None                                      # kein Bild
        now = time.monotonic()                                # aktuelle Zeit
        if frame is None:                                     # kein Bild bekommen
            if now - self._last_good_frame > 5.0:             # seit 5 s nichts → Kamera neu starten
                logger.warning("Kamera liefert keine Bilder – Neustart")  # Warnung
                self.stop_camera()                            # stoppen (Neustart beim nächsten start_camera)
            return None                                       # kein Bild
        self._last_good_frame = now                           # Zeitpunkt merken
        with self._frame_cond:                                # exklusiver Zugriff
            self._frame_raw = frame                           # als letztes Bild speichern
            self._frame_time = now                            # Zeitpunkt speichern
            self._frame_cond.notify_all()                     # wartende Threads wecken (Kalibrierung)
        return frame                                          # Bild zurückgeben

    def wait_frame(self, newer_than, timeout=3.0):            # wartet auf ein Bild, das nach "newer_than" kam
        deadline = time.monotonic() + timeout                 # spätester Zeitpunkt
        with self._frame_cond:                                # exklusiver Zugriff
            while self._frame_raw is None or self._frame_time <= newer_than:  # noch kein passendes Bild
                remaining = deadline - time.monotonic()       # verbleibende Zeit
                if remaining <= 0:                            # Zeit abgelaufen
                    return None                               # kein Bild
                self._frame_cond.wait(remaining)              # warten (wird von capture() geweckt)
            return self._frame_raw.copy()                     # Kopie des Bildes zurückgeben

    # ------------------------------------------------------------------ Muster -------
    def _get_aruco(self):                                     # liefert den ArUco-Detektor (passend zur Konfig)
        name = self.cfg["marker_dictionary"]                  # gewünschtes Wörterbuch
        if self._aruco is None or name != self._aruco_name:   # noch nicht da oder geändert
            try:                                              # ungültige Namen abfangen
                dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))  # Wörterbuch
            except AttributeError:                            # Name unbekannt
                logger.warning(f"Unbekanntes ArUco-Wörterbuch {name}, nehme DICT_4X4_50")  # Warnung
                dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)  # Ersatz
            params = cv2.aruco.DetectorParameters()           # Standard-Parameter
            self._aruco = cv2.aruco.ArucoDetector(dictionary, params)  # Detektor anlegen
            self._aruco_name = name                           # Namen merken
        return self._aruco                                    # Detektor zurückgeben

    def detect_markers(self, frame):                          # sucht ArUco-Marker im Bild
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)        # Graubild
        corners, ids, _rejected = self._get_aruco().detectMarkers(gray)  # Erkennung
        result = []                                           # Ergebnisliste
        if ids is None:                                       # nichts gefunden
            return result                                     # leere Liste
        wanted = self.cfg["marker_id"]                        # gesuchte ID (-1 = jede)
        for c, i in zip(corners, ids.flatten()):              # jeden Marker durchgehen
            if wanted >= 0 and int(i) != wanted:              # falsche ID
                continue                                      # überspringen
            pts = c.reshape(4, 2)                             # 4 Eckpunkte
            cx, cy = pts.mean(axis=0)                         # Mittelpunkt
            area = float(cv2.contourArea(pts.astype(np.float32)))  # Fläche
            result.append(Marker(int(i), (float(cx), float(cy)), pts, area))  # speichern
        result.sort(key=lambda m: m.area, reverse=True)       # größter (nächster) Marker zuerst
        return result                                         # Liste zurückgeben

    # ------------------------------------------------------------------ Objekte ------
    def _get_detector(self):                                  # liefert das KI-Modell (legt es bei Bedarf an)
        if self._detector is None and time.monotonic() >= self._detector_retry:  # fehlt und Versuch erlaubt
            try:                                              # Fehler abfangen (Dienst evtl. noch nicht bereit)
                self._detector = self._detector_factory()     # Modell anlegen
                logger.info("Objekterkennung bereit")         # Info
            except Exception as e:                            # nicht bereit
                logger.warning(f"Objekterkennung noch nicht bereit: {e}")  # Warnung
                self._detector_retry = time.monotonic() + 10.0  # in 10 s erneut
        return self._detector                                 # Modell oder None

    def detect_objects(self, frame):                          # erkennt Personen und Autos im Bild
        detector = self._get_detector()                       # KI-Modell holen
        if detector is None:                                  # nicht verfügbar
            return None                                       # kein Ergebnis
        ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])  # Bild als JPEG
        if not ok:                                            # Kodierung fehlgeschlagen
            return None                                       # kein Ergebnis
        try:                                                  # Fehler der Erkennung abfangen
            out = detector.detect(jpg.tobytes(), image_type="jpg",  # Bild an das Modell schicken
                                  confidence=self.cfg["detection_confidence"])  # mit Mindest-Sicherheit
        except Exception as e:                                # Fehler
            out = None                                        # kein Ergebnis
            if time.monotonic() - self._detect_warn_time > 10:  # nicht zu oft warnen
                logger.warning(f"Objekterkennung fehlgeschlagen: {e}")  # Warnung
                self._detect_warn_time = time.monotonic()     # Zeitpunkt merken
        if not out or "detection" not in out:                 # kein brauchbares Ergebnis
            return None if out is None else []               # None = Fehler, [] = nichts erkannt
        persons = self.cfg.labels("person_labels")            # Klassen für "Person"
        cars = self.cfg.labels("car_labels")                  # Klassen für "Auto"
        h, w = frame.shape[:2]                                # Bildgröße
        result = []                                           # Ergebnisliste
        for obj in out["detection"]:                          # jedes erkannte Objekt
            label = str(obj.get("class_name", "")).lower()    # Klassenname
            box = obj.get("bounding_box_xyxy") or [0, 0, 0, 0]  # Rahmen
            x1, y1, x2, y2 = [float(v) for v in box]          # als Zahlen
            x1, x2 = max(0.0, min(x1, x2)), min(float(w), max(x1, x2))  # in das Bild begrenzen (x)
            y1, y2 = max(0.0, min(y1, y2)), min(float(h), max(y1, y2))  # in das Bild begrenzen (y)
            kind = "person" if label in persons else "car" if label in cars else "other"  # Art bestimmen
            conf = float(obj.get("confidence", 0) or 0)       # Sicherheit in %
            result.append(Detection(label, kind, conf, (x1, y1, x2, y2)))  # speichern
        return result                                         # Liste zurückgeben

    # ------------------------------------------------------------------ Ausgabe ------
    def publish(self, frame, quality=70):                     # legt ein Bild für die Webseite ab
        ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])  # JPEG kodieren
        if ok:                                                # erfolgreich?
            with self._jpeg_lock:                             # exklusiver Zugriff
                self._jpeg = jpg.tobytes()                    # speichern
                self._jpeg_seq += 1                           # Zähler erhöhen

    def get_jpeg(self):                                       # liefert das letzte JPEG
        with self._jpeg_lock:                                 # exklusiver Zugriff
            return self._jpeg_seq, self._jpeg                 # Zähler und Daten

    def publish_placeholder(self, text):                      # Ersatzbild (z.B. "Ruhemodus – Kamera aus")
        w, h = self.cfg["camera_width"], self.cfg["camera_height"]  # Bildgröße
        img = np.zeros((h, w, 3), np.uint8)                   # schwarzes Bild
        img[:] = (40, 30, 20)                                 # dunkler Hintergrund
        size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.9, 2)[0]  # Textgröße
        cv2.putText(img, text, ((w - size[0]) // 2, (h + size[1]) // 2),  # Text mittig
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (200, 200, 200), 2, cv2.LINE_AA)  # Schrift, Farbe
        self.publish(img, quality=60)                         # wie ein Kamerabild ablegen

    @staticmethod
    def annotate(frame, detections=(), markers=(), target=None, selected_box=None, info=""):  # zeichnet Ergebnisse ein
        img = frame.copy()                                    # Original nicht verändern
        h, w = img.shape[:2]                                  # Bildgröße
        cv2.drawMarker(img, (w // 2, h // 2), (255, 255, 255), cv2.MARKER_CROSS, 24, 1)  # Bildmitte
        colors = {"person": (0, 0, 255), "car": (255, 0, 0), "other": (128, 128, 128)}  # Farben (BGR)
        for d in detections:                                  # jedes erkannte Objekt
            x1, y1, x2, y2 = [int(v) for v in d.box]          # Rahmen als Ganzzahlen
            thick = 3 if selected_box is not None and d.box == selected_box else 1  # gewählte Person dicker
            cv2.rectangle(img, (x1, y1), (x2, y2), colors[d.kind], thick)  # Rahmen zeichnen
            cv2.putText(img, f"{d.label} {d.confidence:.0f}%", (x1 + 3, max(14, y1 - 4)),  # Beschriftung
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, colors[d.kind], 1, cv2.LINE_AA)  # Schrift
        for m in markers:                                     # jeder Marker
            cv2.polylines(img, [m.corners.astype(np.int32)], True, (0, 255, 0), 2)  # Umriss in Grün
            cv2.putText(img, f"MARKER {m.marker_id}", (int(m.center[0]) + 6, int(m.center[1])),  # Beschriftung
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)  # Schrift
        if target is not None:                                # Nachführziel vorhanden?
            cv2.circle(img, (int(target[0]), int(target[1])), 8, (0, 255, 255), 2)  # Zielpunkt in Gelb
            cv2.line(img, (w // 2, h // 2), (int(target[0]), int(target[1])), (0, 255, 255), 1)  # Linie zur Mitte
        if info:                                              # Statustext?
            cv2.putText(img, info, (8, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)  # unten links
        return img                                            # fertiges Bild
