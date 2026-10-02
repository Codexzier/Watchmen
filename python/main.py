# =====================================================================================
#  Watchmen – Hauptprogramm (Linux-Seite des Arduino UNO Q)
# =====================================================================================
#  Startet alle Teile und verbindet sie:
#    Config      Einstellungen (JSON in /app/data)
#    Mcu         Kommunikation mit dem Sketch (Servos, Schrittmotor, LEDs, Radar, MPU6050)
#    Vision      Kamera, Muster- und Objekterkennung
#    Controller  Zustandsmaschine (Kalibrierung, Erkennung, Ruhemodus)
#    WebUI       Webseite unter http://<UNO-Q-Adresse>:7000
# =====================================================================================

from arduino.app_utils import App, Logger                     # App-Rahmen und Protokollierung
from arduino.app_bricks.web_ui import WebUI                   # Webseiten-Brick
from arduino.app_bricks.object_detection import ObjectDetection  # Objekterkennungs-Brick (KI-Modell)

from config import Config                                     # Einstellungen
from mcu import Mcu                                           # Sketch-Schnittstelle
from vision import Vision                                     # Kamera und Bildauswertung
from controller import Controller                             # Ablaufsteuerung
import webapi                                                 # Verbindung zur Webseite

logger = Logger("watchmen")                                   # Logger der Anwendung

cfg = Config()                                                # Einstellungen laden
mcu = Mcu()                                                   # Bridge-Funktionen anmelden
ui = WebUI()                                                  # Webserver (Port 7000) anlegen


def make_detector():                                          # legt das KI-Modell an (bei Bedarf, ggf. mehrfach)
    return ObjectDetection(confidence=cfg["detection_confidence"])  # Objekterkennung mit Mindest-Sicherheit


vision = Vision(cfg, make_detector)                           # Kamera/Erkennung vorbereiten
controller = Controller(cfg, mcu, vision, ui)                 # Ablaufsteuerung anlegen
webapi.register(ui, controller, vision, cfg, mcu)             # Webseiten-Funktionen anmelden

logger.info("Watchmen startet")                               # Info ins Protokoll
App.run(user_loop=controller.loop_once)                       # Hauptschleife: ruft loop_once() immer wieder auf
