# =====================================================================================
#  Watchmen – Verbindung zur Webseite (WebUI-Brick)
# =====================================================================================
#  - Socket.IO-Nachrichten: Kalibrierung, Einstellungen, Aufwecken ...
#  - /stream      Live-Bild als MJPEG (für ein <img>-Element)
#  - /marker.png  druckbarer ArUco-Marker (das "markante Muster")
#  - /api/state   aktueller Zustand als JSON
# =====================================================================================

import time                                                   # Wartezeiten im Bild-Stream

from fastapi.responses import Response, StreamingResponse     # HTTP-Antworten (Bild, Stream)

from config import mcu_vector_from                            # Zahlenliste für den Sketch
from controller import STATE_CALIB, STATE_CALIB2_MANUAL        # Zustände mit erlaubter Handsteuerung
from vision import marker_png                                 # Marker-Bild erzeugen


def register(ui, controller, vision, cfg, mcu):               # meldet alle Funktionen bei der Webseite an
    def queue_cmd(name):                                      # erzeugt eine Funktion, die einen Befehl einreiht
        def handler(sid, data):                               # wird bei der Nachricht aufgerufen
            controller.commands.put((name, data if isinstance(data, dict) else {}))  # an die Hauptschleife geben
            return {"ok": True}                               # Bestätigung an die Webseite
        return handler                                        # Funktion zurückgeben

    for name in ("cal_enter", "cal_cancel", "cal_save1", "cal_start2", "cal_save2",  # Befehle, die den Zustand ändern
                 "cal_reset", "settings_save", "wake", "sleep", "retry_homing"):  # ...
        ui.on_message(name, queue_cmd(name))                  # anmelden

    def in_calibration():                                     # sind Handbewegungen erlaubt?
        return controller.state in (STATE_CALIB, STATE_CALIB2_MANUAL)  # nur in der Kalibrierung, nicht während der Automatik

    def cal_tilt(sid, data):                                  # Schieberegler Servo 1
        if in_calibration():                                  # nur in der Kalibrierung
            mcu.tilt(float(data.get("angle", 90)), raw=True)  # Servo 1 stellen (voller Bereich)

    def cal_radar(sid, data):                                 # Schieberegler Servo 2
        if in_calibration():                                  # nur in der Kalibrierung
            angle = data.get("angle")                         # Winkel oder None
            mcu.radar_servo(None if angle is None else float(angle))  # stellen oder Automatik

    def cal_jog(sid, data):                                   # Drehachse ein Stück fahren
        if in_calibration():                                  # nur in der Kalibrierung
            mcu.pan_jog(int(data.get("steps", 0)))            # relativ fahren

    def cal_goto(sid, data):                                  # Drehachse auf Position fahren
        if in_calibration():                                  # nur in der Kalibrierung
            mcu.pan_to(int(data.get("pos", 0)))               # absolut fahren

    def cal_home(sid, data):                                  # Referenzfahrt (Kontaktschalter anfahren)
        if in_calibration():                                  # nur in der Kalibrierung
            mcu.home()                                        # starten

    def cal_stop(sid, data):                                  # Drehachse anhalten
        mcu.stop()                                            # immer erlaubt

    def cal_preview(sid, data):                               # Kalibrierwerte sofort testen (ohne Speichern)
        if in_calibration():                                  # nur in der Kalibrierung
            keys = ("radar_level", "radar_comp_factor", "radar_comp_enabled", "stepper_invert",  # erlaubte Werte
                    "servo_us_min", "servo_us_max", "tilt_level", "radar_min", "radar_max")  # ...
            preview = cfg.snapshot()                          # aktuelle Werte
            for k in keys:                                    # neue Werte eintragen
                if k in data:                                 # vorhanden?
                    v = cfg.coerce(k, data[k])                # prüfen
                    if v is not None:                         # gültig?
                        preview[k] = v                        # übernehmen
            preview["calib1_done"] = False                    # beim Testen keine Grenzen
            vector = mcu_vector_from(preview)                 # Zahlenliste für den Sketch
            mcu.send_config(vector)                           # nur an den Sketch (nicht speichern)

    for name, fn in (("cal_tilt", cal_tilt), ("cal_radar", cal_radar), ("cal_jog", cal_jog),  # direkte Befehle
                     ("cal_goto", cal_goto), ("cal_home", cal_home), ("cal_stop", cal_stop),  # ...
                     ("cal_preview", cal_preview)):           # ...
        ui.on_message(name, fn)                               # anmelden

    def on_connect(sid):                                      # neuer Browser verbunden
        controller.send_config()                              # Einstellungen schicken
    ui.on_connect(on_connect)                                 # anmelden

    def stream():                                             # Live-Bild als MJPEG
        def frames():                                         # erzeugt die Einzelbilder
            last = -1                                         # zuletzt gesendetes Bild
            while True:                                       # endlos (bis der Browser trennt)
                seq, jpg = vision.get_jpeg()                  # neuestes Bild
                if jpg is None or seq == last:                # nichts Neues
                    time.sleep(0.03)                          # kurz warten
                    continue                                  # erneut prüfen
                last = seq                                    # merken
                yield (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "  # Kopf des Einzelbildes
                       + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")  # Bilddaten
                time.sleep(0.05)                              # höchstens ca. 20 Bilder/s
        return StreamingResponse(frames(), media_type="multipart/x-mixed-replace; boundary=frame")  # Antwort
    ui.expose_api("GET", "/stream", stream)                   # Adresse /stream

    def marker(id: int = 0, dictionary: str = ""):            # druckbarer Marker (z.B. /marker.png?id=3)
        name = dictionary or cfg["marker_dictionary"]         # Wörterbuch (Standard aus Konfig)
        try:                                                  # ungültige Eingaben abfangen
            png = marker_png(name, max(0, int(id)))           # Bild erzeugen
        except Exception:                                     # Fehler
            png = b""                                         # leeres Ergebnis
        if not png:                                           # nichts erzeugt
            return Response(content=b"Marker nicht erzeugbar", media_type="text/plain", status_code=400)  # Fehler
        return Response(content=png, media_type="image/png")  # PNG zurückgeben
    ui.expose_api("GET", "/marker.png", marker)               # Adresse /marker.png

    def api_state():                                          # Zustand als JSON
        return controller.get_snapshot()                      # letzter Zustand
    ui.expose_api("GET", "/api/state", api_state)             # Adresse /api/state

    def api_config():                                         # Einstellungen als JSON
        return cfg.snapshot()                                 # alle Werte
    ui.expose_api("GET", "/api/config", api_config)           # Adresse /api/config

