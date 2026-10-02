"""
Simulations-Server: startet die echte Watchmen-Steuerung mit simulierter Hardware und
der echten Webseite (assets/). So lässt sich die Oberfläche am PC ausprobieren.

Aufruf:   python3 tests/python/sim_server.py [--port 7000]
Danach:   http://localhost:7000 im Browser öffnen.

Benötigt: numpy, opencv-python-headless, fastapi, uvicorn, python-socketio
"""
import argparse
import asyncio
import math
import os
import sys
import tempfile
import threading
import time

import socketio
import uvicorn
from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles

HERE = os.path.dirname(os.path.abspath(__file__))
ASSETS = os.path.join(HERE, "..", "..", "assets")
sys.path.insert(0, HERE)

import test_system as ts  # noqa: E402  (World, SimMcu, Fakes, Pfade)
from arduino.app_utils import Bridge  # noqa: E402
import arduino.app_bricks.object_detection as od_fake  # noqa: E402
import arduino.app_peripherals.camera as cam_fake  # noqa: E402
import config as config_mod  # noqa: E402
from mcu import Mcu  # noqa: E402
from vision import Vision  # noqa: E402
import controller as ctl  # noqa: E402
import webapi  # noqa: E402
from arduino.app_bricks.object_detection import ObjectDetection  # noqa: E402


class SimWebUI:
    """Gleiche Schnittstelle wie der WebUI-Brick (on_message, send_message, expose_api ...)."""

    def __init__(self, assets):
        self.assets = assets
        self.sio = socketio.AsyncServer(async_mode="asgi", cors_allowed_origins="*")
        self.api = FastAPI()
        self.handlers = {}
        self.connect_cb = None
        self.loop = None

        @self.sio.on("connect")
        async def _connect(sid, environ, auth=None):
            if self.connect_cb:
                await asyncio.to_thread(self.connect_cb, sid)

        @self.sio.on("*")
        async def _any(event, sid, data=None):
            fn = self.handlers.get(event)
            if fn is None:
                return
            res = await asyncio.to_thread(fn, sid, data if data is not None else {})
            if res is not None:
                await self.sio.emit(event + "_response", res, to=sid)

        @self.api.on_event("startup")
        async def _startup():
            self.loop = asyncio.get_running_loop()

    def on_message(self, name, fn):
        self.handlers[name] = fn

    def on_connect(self, fn):
        self.connect_cb = fn

    def expose_api(self, method, path, fn):
        self.api.add_api_route(path, fn, methods=[method])

    def send_message(self, name, message, room=None):
        if self.loop is not None and self.loop.is_running():
            asyncio.run_coroutine_threadsafe(self.sio.emit(name, message), self.loop)

    def app(self):
        self.api.mount("/", StaticFiles(directory=self.assets, html=True), name="static")
        return socketio.ASGIApp(self.sio, other_asgi_app=self.api, socketio_path="socket.io")


def radar_feeder(world, sim, stop):
    """Erzeugt Radar-Ziele aus den simulierten Personen (Abstand je Person in world.person_dist)."""
    half = math.radians(31.1)
    while not stop.is_set():
        time.sleep(0.1)
        x0, y0 = world.view_origin()
        vals = []
        for i, (wx, wy, w, h) in enumerate(world.persons[:3]):
            img_x = wx + w / 2 - x0
            ang = math.atan(math.tan(half) * (img_x - 320) / 320)
            if world.ceiling:
                ang = -ang
            d = world.person_dist[i] if i < len(world.person_dist) else 2500
            vals += [int(d * math.sin(ang)), int(d * math.cos(ang)), 12]
        vals += [0] * (9 - len(vals))
        Bridge.notify("mcu_radar", *vals)


def actor(world, stop):
    """Lässt eine Person langsam durch die Szene laufen (nach der Kalibrierung)."""
    t0 = time.monotonic()
    while not stop.is_set():
        time.sleep(0.05)
        t = time.monotonic() - t0
        if world.walking:
            cx = 2100 + 260 * math.sin(t / 6.0)
            world.persons = [(int(cx), 1150, 90, 210), (int(cx) + 330, 1180, 80, 190)]
            world.person_dist = [1800, 4200]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=7000)
    ap.add_argument("--calibrated", action="store_true", help="mit fertiger Kalibrierung starten")
    args = ap.parse_args()

    world = ts.World()
    world.walking = False
    world.person_dist = []
    sim = ts.SimMcu()
    world.sim = sim
    Bridge.target = sim
    cam_fake.HOOK["render"] = world.render
    od_fake.HOOK["fn"] = world.detections

    tmp = tempfile.mkdtemp(prefix="watchmen-sim-")
    cfg = config_mod.Config(os.path.join(tmp, "cfg.json"))
    cfg.update({"camera_fps": 12, "sleep_timeout_s": 120})
    if args.calibrated:
        cfg.update(dict(ts.CALIB1, calib1_done=True, calib2_done=True, pan_dir=1, tilt_dir=1))
    mcu = Mcu()
    ui = SimWebUI(os.path.abspath(ASSETS))
    vision = Vision(cfg, lambda: ObjectDetection(confidence=cfg["detection_confidence"]))
    c = ctl.Controller(cfg, mcu, vision, ui)
    webapi.register(ui, c, vision, cfg, mcu)

    stop = threading.Event()

    def loop():
        while not stop.is_set():
            c.loop_once()
            if c.state == ctl.STATE_ACTIVE:
                world.walking = True

    threading.Thread(target=loop, daemon=True).start()
    threading.Thread(target=radar_feeder, args=(world, sim, stop), daemon=True).start()
    threading.Thread(target=actor, args=(world, stop), daemon=True).start()
    print(f"Simulation läuft: http://localhost:{args.port}  (Konfiguration in {tmp})", flush=True)
    uvicorn.run(ui.app(), host="127.0.0.1", port=args.port, log_level="warning")
    stop.set()


if __name__ == "__main__":
    main()
