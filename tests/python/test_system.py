"""
Systemtest der Python-Seite (läuft am PC, ohne UNO Q).

Simuliert:
  - den Mikrocontroller (Bridge-Aufrufe, Statusmeldungen, Referenzfahrt, Motorbewegung),
  - eine Kamera, die einen Ausschnitt aus einer Textur-"Welt" zeigt (abhängig von Drehung/Neigung),
  - die KI-Objekterkennung (liefert Personen/Autos aus der simulierten Welt),
  - ArUco-Marker (werden echt mit OpenCV gezeichnet und erkannt).

Aufruf:  python3 tests/python/test_system.py      (benötigt numpy, opencv-python-headless, fastapi)
"""
import math
import os
import sys
import tempfile
import threading
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "fakes"))
sys.path.insert(0, os.path.join(HERE, "..", "..", "python"))

from arduino.app_utils import Bridge  # noqa: E402  (Fake)
from arduino.app_bricks.web_ui import WebUI  # noqa: E402  (Fake)
import arduino.app_bricks.object_detection as od_fake  # noqa: E402
import arduino.app_peripherals.camera as cam_fake  # noqa: E402

import config as config_mod  # noqa: E402
from mcu import Mcu  # noqa: E402
from vision import Vision  # noqa: E402
import controller as ctl  # noqa: E402
import webapi  # noqa: E402
from arduino.app_bricks.object_detection import ObjectDetection  # noqa: E402

FAILS = []


def check(cond, text):
    print(("  OK   " if cond else "  FAIL ") + text, flush=True)
    if not cond:
        FAILS.append(text)


def wait_until(fn, timeout, step=0.05):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if fn():
            return True
        time.sleep(step)
    return fn()


# ------------------------------------------------------------------ simulierte Welt ----
STEPS_PER_DEG = 4096 / 360.0
K_PAN = 640 / (62.2 * STEPS_PER_DEG)      # Pixel pro Halbschritt
K_TILT = 480 / 48.8                        # Pixel pro Grad


class World:
    def __init__(self):
        rng = np.random.default_rng(7)
        noise = rng.random((2400, 4200)).astype(np.float32)
        noise = cv2.GaussianBlur(noise, (0, 0), 4)
        noise = cv2.normalize(noise, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        self.texture = cv2.cvtColor(noise, cv2.COLOR_GRAY2BGR)
        self.blank = np.full_like(self.texture, 128)
        self.use_blank = False
        self.pan_sign = 1
        self.tilt_sign = 1
        self.ceiling = False
        self.persons = []     # (wx, wy, w, h) in Weltkoordinaten
        self.cars = []
        self.marker_at = None  # (wx, wy)
        d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        m = cv2.aruco.generateImageMarker(d, 3, 120)
        self.marker_img = cv2.cvtColor(cv2.copyMakeBorder(m, 20, 20, 20, 20, cv2.BORDER_CONSTANT, value=255), cv2.COLOR_GRAY2BGR)
        self.sim = None

    def view_origin(self):
        s = -1 if self.ceiling else 1
        cx = 2100 + s * self.pan_sign * (self.sim.pan_pos - 1500) * K_PAN
        cy = 1200 + s * self.tilt_sign * (self.sim.tilt - 90.0) * K_TILT
        return int(round(cx - 320)), int(round(cy - 240))

    def render(self, resolution):
        x0, y0 = self.view_origin()
        src = self.blank if self.use_blank else self.texture
        view = src[y0:y0 + 480, x0:x0 + 640].copy()
        if self.marker_at is not None:
            mx, my = self.marker_at[0] - x0, self.marker_at[1] - y0
            h, w = self.marker_img.shape[:2]
            if 0 <= mx and mx + w <= 640 and 0 <= my and my + h <= 480:
                view[my:my + h, mx:mx + w] = self.marker_img
        return cv2.rotate(view, cv2.ROTATE_180) if self.ceiling else view

    def detections(self):
        x0, y0 = self.view_origin()
        out = []
        for kind, items in (("person", self.persons), ("car", self.cars)):
            for (wx, wy, w, h) in items:
                x1, y1 = wx - x0, wy - y0
                if x1 + w < 0 or y1 + h < 0 or x1 > 640 or y1 > 480:
                    continue
                out.append({"class_name": kind, "confidence": "90.00",
                            "bounding_box_xyxy": [max(0, x1), max(0, y1), min(640, x1 + w), min(480, y1 + h)]})
        return out


# ------------------------------------------------------------------ simulierter Sketch --
class SimMcu:
    def __init__(self):
        self.lock = threading.Lock()
        self.motors_present = True
        self.config = None
        self.homed = False
        self.homing_state = 0
        self.homing_start = 0.0
        self.pan_pos = 1500.0
        self.pan_target = 1500.0
        self.tilt = 90.0
        self.tilt_target = 90.0
        self.radar = 90.0
        self.radar_manual = False
        self.acc = [0, 0, 1000]
        self.leds = {0: (0, 0), 1: (0, 0)}
        self.text = ""
        self.flashes = 0
        self.pan_cmds = 0
        self.running = True
        threading.Thread(target=self._run, daemon=True).start()

    def _cfg(self, i, default):
        return self.config[i] if self.config else default

    def _run(self):
        last = time.monotonic()
        last_status = 0.0
        while self.running:
            time.sleep(0.01)
            now = time.monotonic()
            dt = now - last
            last = now
            with self.lock:
                if self.homing_state == 1 and now - self.homing_start > 0.3:
                    self.pan_pos = self.pan_target = 40.0
                    self.homed = True
                    self.homing_state = 3
                    Bridge.notify("mcu_event", 1, 40)
                if self.motors_present:
                    step = 3000 * dt
                    d = self.pan_target - self.pan_pos
                    self.pan_pos += max(-step, min(step, d))
                    t = 300 * dt
                    d = self.tilt_target - self.tilt
                    self.tilt += max(-t, min(t, d))
                    if not self.radar_manual and self.config:
                        lvl, comp = self.config[5] / 10, self.config[6] / 100
                        self.radar = lvl + comp * (self.tilt - self.config[2] / 10)
            if now - last_status > 0.05:
                last_status = now
                with self.lock:
                    vals = [int(self.motors_present), int(self.config is not None), int(self.homed), self.homing_state,
                            int(round(self.pan_pos)), int(abs(self.pan_target - self.pan_pos) > 0.5),
                            int(round(self.tilt * 10)), int(round(self.radar * 10)), 0,
                            self.acc[0], self.acc[1], self.acc[2], 0, 1, 1]
                Bridge.notify("mcu_status", *vals)

    # RPC-Funktionen (Namen wie im Sketch)
    def rpc_set_config(self, v):
        with self.lock:
            assert len(v) == 20
            first = self.config is None
            self.config = list(v)
            if first:
                self.tilt = self.tilt_target = v[2] / 10
        return True

    def rpc_tilt(self, deg10, raw):
        with self.lock:
            if not self.motors_present:
                return False
            d = deg10 / 10
            if not raw and self.config and self.config[16]:
                d = max(self.config[0] / 10, min(self.config[1] / 10, d))
            self.tilt_target = d
        return True

    def rpc_radar_servo(self, deg10):
        with self.lock:
            self.radar_manual = deg10 >= 0
            if deg10 >= 0:
                self.radar = deg10 / 10
        return True

    def rpc_pan_to(self, pos):
        with self.lock:
            if not (self.motors_present and self.homed):
                return False
            self.pan_cmds += 1
            if self.config and self.config[16]:
                pos = max(self.config[8], min(self.config[9], pos))
            self.pan_target = float(pos)
        return True

    def rpc_pan_jog(self, delta):
        with self.lock:
            self.pan_target += delta
        return True

    def rpc_home(self):
        with self.lock:
            self.homed = False
            self.homing_state = 1
            self.homing_start = time.monotonic()
        return True

    def rpc_stop(self):
        with self.lock:
            self.pan_target = self.pan_pos
        return True

    def rpc_led(self, idx, rgb, blink):
        with self.lock:
            for i in ((0, 1) if idx == 2 else (idx,)):
                self.leds[i] = (rgb, blink)
        return True

    def rpc_flash(self, rgb, count):
        self.flashes += 1
        return True

    def rpc_text(self, s):
        self.text = s
        return True

    def rpc_version(self):
        return 1


# ------------------------------------------------------------------ Aufbau ------------
def build(tmpdir, world):
    sim = SimMcu()
    world.sim = sim
    Bridge.target = sim
    cam_fake.HOOK["render"] = world.render
    od_fake.HOOK["fn"] = world.detections
    cfg = config_mod.Config(os.path.join(tmpdir, "cfg.json"))
    cfg.update({"sleep_timeout_s": 4, "camera_fps": 20, "hold_time_s": 0.5, "track_interval_s": 0.1})
    mcu = Mcu()
    ui = WebUI()
    vision = Vision(cfg, lambda: ObjectDetection(confidence=cfg["detection_confidence"]))
    c = ctl.Controller(cfg, mcu, vision, ui)
    webapi.register(ui, c, vision, cfg, mcu)
    stop = threading.Event()

    def loop():
        while not stop.is_set():
            c.loop_once()
    threading.Thread(target=loop, daemon=True).start()
    return sim, cfg, ui, c, stop


CALIB1 = {"tilt_min": 45, "tilt_max": 135, "tilt_level": 90, "radar_min": 30, "radar_max": 150, "radar_level": 90,
          "radar_comp_factor": -1, "radar_comp_enabled": True, "pan_min": 20, "pan_max": 3000, "pan_center": 1500,
          "stepper_invert": False, "servo_us_min": 500, "servo_us_max": 2500, "homing_max_steps": 5000}


def person_error(world, c):
    dets = world.detections()
    ps = [d for d in dets if d["class_name"] == "person"]
    if not ps:
        return None
    x1, y1, x2, y2 = ps[0]["bounding_box_xyxy"]
    cx, cy = (x1 + x2) / 2, y1 + 0.35 * (y2 - y1)
    return abs(cx - 320) / 640, abs(cy - 240) / 480


def scenario_main(pan_sign, tilt_sign):
    print(f"\n=== Szenario: Drehrichtung {pan_sign:+d}, Neigungsrichtung {tilt_sign:+d} ===")
    world = World()
    world.pan_sign, world.tilt_sign = pan_sign, tilt_sign
    with tempfile.TemporaryDirectory() as tmp:
        sim, cfg, ui, c, stop = build(tmp, world)
        try:
            check(wait_until(lambda: c.state == ctl.STATE_CALIB, 5), "Start: nicht kalibriert → Kalibrierung 1")
            check(wait_until(lambda: sim.leds[0] == (0xFF0000, 500) and sim.leds[1] == (0xFF0000, 500), 2), "beide LEDs blinken rot")
            ui.emit("cal_tilt", {"angle": 120})
            check(wait_until(lambda: abs(sim.tilt - 120) < 0.5, 3), "Schieberegler stellt Servo 1 (Kalibrierung)")
            ui.emit("cal_tilt", {"angle": 90})
            bad = dict(CALIB1, tilt_min=100)
            ui.emit("cal_save1", bad)
            time.sleep(0.5)
            check(not cfg["calib1_done"] and ui.last("calib_error"), "ungültige Bereiche werden abgelehnt")
            ui.emit("cal_save1", CALIB1)
            check(wait_until(lambda: cfg["calib1_done"], 3), "Kalibrierung 1 gespeichert")
            ui.emit("cal_start2", {})
            check(wait_until(lambda: c.state == ctl.STATE_CALIB2, 3), "Kalibrierung 2 gestartet")
            check(wait_until(lambda: c.state == ctl.STATE_CALIB_DONE, 40), "Kalibrierung 2 automatisch erfolgreich")
            check(cfg["pan_dir"] == pan_sign and cfg["tilt_dir"] == tilt_sign,
                  f"Richtungen erkannt (pan_dir={cfg['pan_dir']}, tilt_dir={cfg['tilt_dir']})")
            exp_pan = -pan_sign / (62.2 * STEPS_PER_DEG)
            check(abs(cfg["pan_frac_per_step"] - exp_pan) / abs(exp_pan) < 0.15,
                  f"Skalierung Drehen ±15 % ({cfg['pan_frac_per_step']:.6f} statt {exp_pan:.6f})")
            exp_tilt = -tilt_sign / 48.8
            check(abs(cfg["tilt_frac_per_deg"] - exp_tilt) / abs(exp_tilt) < 0.15,
                  f"Skalierung Neigen ±15 % ({cfg['tilt_frac_per_deg']:.5f} statt {exp_tilt:.5f})")
            check(sim.leds[0] == (0x00FF00, 0) and sim.leds[1] == (0x00FF00, 0), "beide LEDs grün")
            t0 = time.monotonic()
            check(wait_until(lambda: c.state == ctl.STATE_ACTIVE, 7), "nach ca. 5 s Objekterkennung aktiv")
            green = time.monotonic() - t0
            check(3.5 < green < 6.0, f"Grünphase ca. 5 s ({green:.1f} s)")

            print("  -- Person verfolgen")
            time.sleep(0.5)
            x0, y0 = world.view_origin()
            world.persons = [(x0 + 470, y0 + 260, 80, 200)]
            check(wait_until(lambda: sim.text == "HUMEN" and sim.leds[0] == (0xFF0000, 0), 3),
                  "Person: Matrix 'HUMEN', LED 1 rot")
            check(wait_until(lambda: (person_error(world, c) or (1, 1))[0] < 0.06 and
                             (person_error(world, c) or (1, 1))[1] < 0.07, 8), "Kamera folgt der Person (zentriert)")

            print("  -- Auto")
            world.persons = []
            x0, y0 = world.view_origin()
            world.cars = [(x0 + 100, y0 + 300, 160, 90)]
            n_before = sim.pan_cmds
            check(wait_until(lambda: sim.text == "CAR" and sim.leds[0] == (0x0000FF, 0), 3), "Auto: 'CAR', LED 1 blau")
            time.sleep(0.5)
            check(sim.pan_cmds == n_before, "Auto wird nicht verfolgt")

            print("  -- Muster (Marker) hat Vorrang")
            x0, y0 = world.view_origin()
            world.marker_at = (x0 + 380, y0 + 60)
            check(wait_until(lambda: sim.text == "MARK" and sim.leds[0] == (0xFFFFFF, 0), 3), "Muster: 'MARK', LED 1 weiß")

            def marker_err():
                x0, y0 = world.view_origin()
                mx = world.marker_at[0] - x0 + 80
                my = world.marker_at[1] - y0 + 80
                return abs(mx - 320) / 640 < 0.06 and abs(my - 240) / 480 < 0.07
            check(wait_until(marker_err, 8), "Kamera hält das Muster in der Bildmitte")

            print("  -- Ruhemodus")
            world.marker_at, world.cars = None, []
            stops = cam_fake.HOOK["stops"]
            check(wait_until(lambda: c.state == ctl.STATE_SLEEP, 8), "nach Zeitablauf Ruhemodus")
            check(cam_fake.HOOK["stops"] > stops and not c.vision.camera_running(), "Kamera abgeschaltet")
            check(sim.leds[0] == (0, 0) and sim.leds[1] == (0xFF6000, 500), "LED 1 aus, LED 2 blinkt orange (1 Hz)")
            check(wait_until(lambda: abs(sim.tilt - 90) < 0.5 and abs(sim.radar - 90) < 0.5, 3), "Servos waagerecht")
            check(sim.text == "", "Matrix leer")

            print("  -- Aufwachen per Radar")
            Bridge.notify("mcu_radar", 300, 4500, 30, 0, 0, 0, 0, 0, 0)  # zu weit weg
            time.sleep(0.3)
            check(c.state == ctl.STATE_SLEEP, "Ziel außerhalb des Weckabstands weckt nicht")
            for _ in range(3):
                Bridge.notify("mcu_radar", 300, 1500, 30, 0, 0, 0, 0, 0, 0)
                time.sleep(0.12)
            check(wait_until(lambda: c.state == ctl.STATE_ACTIVE, 2), "Radar-Ziel (1,5 m, bewegt) weckt auf")
            check(c.vision.camera_running(), "Kamera wieder an")

            print("  -- Aufwachen per Vibration")
            check(wait_until(lambda: c.state == ctl.STATE_SLEEP, 8), "wieder Ruhemodus")
            Bridge.notify("mcu_vibration", 400)
            check(wait_until(lambda: c.state == ctl.STATE_ACTIVE, 2), "Vibration weckt auf")

            print("  -- Mehrere Personen + Radar: nähere Person wird verfolgt")
            x0, y0 = world.view_origin()
            world.persons = [(x0 + 60, y0 + 200, 70, 180), (x0 + 480, y0 + 200, 70, 180)]
            # linke Person (Bild-x ~95) ist näher: 1,5 m; rechte (Bild-x ~515) 4 m
            def radar_xy(img_x, dist):
                rel = (img_x - 320) / 320 * math.tan(math.radians(31.1))
                ang = math.atan(rel)
                return int(dist * math.sin(ang)), int(dist * math.cos(ang))
            lx, ly = radar_xy(95, 1500)
            rx, ry = radar_xy(515, 4000)
            sx = -1 if world.ceiling else 1
            stop_radar = threading.Event()

            def radar_feed():
                while not stop_radar.is_set():
                    Bridge.notify("mcu_radar", rx * sx, ry, 5, lx * sx, ly, 5, 0, 0, 0)
                    time.sleep(0.1)
            threading.Thread(target=radar_feed, daemon=True).start()
            check(wait_until(lambda: c.radar_selected == 1, 4), "Radar-Ziel der näheren Person ausgewählt")

            def left_centered():
                x0, y0 = world.view_origin()
                cx = world.persons[0][0] - x0 + 35
                return abs(cx - 320) / 640 < 0.08
            check(wait_until(left_centered, 8), "Kamera dreht zur näheren (linken) Person")
            stop_radar.set()
            world.persons = []

            print("  -- Deckenmontage")
            world.ceiling = True
            sim.acc = [0, 0, -1000]
            check(wait_until(lambda: c.flipped, 3), "MPU6050 erkennt Deckenmontage → Bild gedreht")
            time.sleep(0.5)
            x0, y0 = world.view_origin()
            world.persons = [(x0 + 120, y0 + 120, 80, 200)]
            check(wait_until(lambda: (person_error(world, c) or (1, 1))[0] < 0.06 and
                             (person_error(world, c) or (1, 1))[1] < 0.07, 10), "Verfolgung funktioniert auch kopfüber")
            world.persons = []
            world.ceiling = False
            sim.acc = [0, 0, 1000]
            check(wait_until(lambda: not c.flipped, 3), "zurück auf normale Montage")

            print("  -- Motoren abgesteckt (A0)")
            with sim.lock:
                sim.motors_present = False
            check(wait_until(lambda: not c.motion_enabled(c.mcu.get_status()), 2), "Bewegung gesperrt")
            n_before = sim.pan_cmds
            x0, y0 = world.view_origin()
            world.persons = [(x0 + 500, y0 + 200, 80, 200)]
            check(wait_until(lambda: sim.text == "HUMEN", 3), "Erkennung läuft ohne Motoren weiter")
            time.sleep(0.6)
            check(sim.pan_cmds == n_before, "keine Fahrbefehle ohne Motoren")
            world.persons = []
            with sim.lock:
                sim.motors_present = True
                sim.homed = False
            check(wait_until(lambda: c.state == ctl.STATE_HOMING, 3), "Motoren wieder da → Referenzfahrt")
            check(wait_until(lambda: c.state == ctl.STATE_ACTIVE, 5), "nach Referenzfahrt wieder aktiv")

            print("  -- Neustart: Konfiguration bleibt, Referenzfahrt beim Start")
        finally:
            stop.set()
            sim.running = False
            time.sleep(0.3)
        cfg_path = os.path.join(tmp, "cfg.json")
        check(os.path.exists(cfg_path), "Konfigurationsdatei geschrieben")
        world2 = World()
        world2.pan_sign, world2.tilt_sign = pan_sign, tilt_sign
        sim, cfg2, ui, c, stop = build(tmp, world2)
        try:
            check(cfg2["calib1_done"] and cfg2["calib2_done"], "Kalibrierung nach Neustart geladen")
            check(wait_until(lambda: c.state == ctl.STATE_HOMING, 5), "nach Neustart: Referenzfahrt")
            check(wait_until(lambda: c.state == ctl.STATE_ACTIVE, 5), "danach Objekterkennung")
        finally:
            stop.set()
            sim.running = False
            time.sleep(0.3)


def scenario_manual():
    print("\n=== Szenario: Kalibrierung 2 schlägt fehl (einfarbige Wand) → von Hand ===")
    world = World()
    world.use_blank = True
    with tempfile.TemporaryDirectory() as tmp:
        sim, cfg, ui, c, stop = build(tmp, world)
        try:
            check(wait_until(lambda: c.state == ctl.STATE_CALIB, 5), "Kalibrierung 1")
            ui.emit("cal_save1", CALIB1)
            check(wait_until(lambda: cfg["calib1_done"], 3), "Kalibrierung 1 gespeichert")
            ui.emit("cal_start2", {})
            check(wait_until(lambda: c.state == ctl.STATE_CALIB2_MANUAL, 40), "Automatik scheitert → Handeingabe")
            check(sim.leds[0] == (0xFF0000, 500), "LEDs blinken weiter rot (nicht kalibriert)")
            ui.emit("cal_save2", {"pan_dir": -1, "tilt_dir": 1})
            check(wait_until(lambda: c.state == ctl.STATE_CALIB_DONE, 3), "Speichern → kalibriert (grün)")
            check(cfg["calib2_done"] and cfg["pan_dir"] == -1 and cfg["calib2_method"] == "manuell", "Werte gespeichert")
            exp = 1 / (62.2 * STEPS_PER_DEG)
            check(abs(cfg["pan_frac_per_step"] - exp) < 1e-9, "Umrechnung aus Bildwinkel berechnet")
            check(wait_until(lambda: c.state == ctl.STATE_ACTIVE, 7), "danach aktiv")
        finally:
            stop.set()
            sim.running = False
            time.sleep(0.3)


def scenario_no_motors():
    print("\n=== Szenario: ohne Motoren starten (A0 = 0 V) ===")
    world = World()
    with tempfile.TemporaryDirectory() as tmp:
        sim0 = None
        orig = SimMcu.__init__

        def init_no_motors(self):
            orig(self)
            self.motors_present = False
        SimMcu.__init__ = init_no_motors
        try:
            sim, cfg, ui, c, stop = build(tmp, world)
        finally:
            SimMcu.__init__ = orig
        try:
            check(wait_until(lambda: c.state == ctl.STATE_ACTIVE, 5), "direkt Objekterkennung (keine Kalibrierung)")
            check(sim.leds[0] != (0xFF0000, 500), "keine rot blinkenden LEDs")
            ui.emit("cal_enter", {})
            time.sleep(0.5)
            check(c.state == ctl.STATE_ACTIVE, "Kalibrierung ohne Motoren nicht möglich")
            world.cars = [(2100 - 60, 1200 - 40, 120, 80)]
            check(wait_until(lambda: sim.text == "CAR", 3), "Auto wird angezeigt")
            world.cars = []
            check(wait_until(lambda: c.state == ctl.STATE_SLEEP, 8), "Ruhemodus auch ohne Motoren")
            del sim0
        finally:
            stop.set()
            sim.running = False
            time.sleep(0.3)


def scenario_web_routes():
    print("\n=== Szenario: Web-Routen ===")
    world = World()
    with tempfile.TemporaryDirectory() as tmp:
        sim, cfg, ui, c, stop = build(tmp, world)
        try:
            resp = ui.routes[("GET", "/marker.png")](id=5)
            check(resp.media_type == "image/png" and resp.body[:4] == b"\x89PNG", "Marker-PNG wird erzeugt")
            img = cv2.imdecode(np.frombuffer(resp.body, np.uint8), cv2.IMREAD_GRAYSCALE)
            det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
            _, ids, _ = det.detectMarkers(img)
            check(ids is not None and int(ids[0][0]) == 5, "gedruckter Marker ist erkennbar (ID 5)")
            check(isinstance(ui.routes[("GET", "/api/config")](), dict), "/api/config liefert JSON")
            check(wait_until(lambda: ui.last("state") is not None, 3), "Status wird an die Webseite gesendet")
            check(wait_until(lambda: ui.last("radar") is not None, 3), "Radar wird an die Webseite gesendet")
            ui.emit("settings_save", {"vibration_threshold_mg": 300, "text_person": "MENSCH", "bogus": 1})
            check(wait_until(lambda: cfg["vibration_threshold_mg"] == 300 and cfg["text_person"] == "MENSCH", 2),
                  "Einstellungen gespeichert, unbekannte ignoriert")
            check(wait_until(lambda: sim.config and sim.config[13] == 300, 3), "Vibrationsschwelle an den Sketch übertragen")
        finally:
            stop.set()
            sim.running = False
            time.sleep(0.3)


if __name__ == "__main__":
    t = time.monotonic()
    scenario_web_routes()
    scenario_main(+1, +1)
    scenario_main(-1, -1)
    scenario_manual()
    scenario_no_motors()
    print(f"\nDauer {time.monotonic() - t:.0f} s")
    if FAILS:
        print(f"{len(FAILS)} FEHLER:")
        for f in FAILS:
            print("  - " + f)
        sys.exit(1)
    print("ALLE TESTS BESTANDEN")
