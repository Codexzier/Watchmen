# Test-Ersatz für die Kamera: Bilder kommen aus einer Funktion des Tests.
import time

HOOK = {"render": None, "starts": 0, "stops": 0}


class _FakeCamera:
    def __init__(self, source, resolution, fps):
        self.resolution = resolution
        self.fps = fps
        self.started = False
        self._last = 0.0

    def start(self):
        self.started = True
        HOOK["starts"] += 1

    def stop(self):
        self.started = False
        HOOK["stops"] += 1

    def capture(self):
        if not self.started:
            raise RuntimeError("nicht gestartet")
        wait = 1.0 / self.fps - (time.monotonic() - self._last)
        if wait > 0:
            time.sleep(wait)
        self._last = time.monotonic()
        return HOOK["render"](self.resolution)


def Camera(source=None, resolution=(640, 480), fps=10, adjustments=None, **kwargs):
    return _FakeCamera(source, resolution, fps)
