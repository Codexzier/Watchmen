# Test-Ersatz für arduino.app_utils: Bridge (an ein simuliertes MCU-Modell gekoppelt), Logger, App.
import logging
import threading


class Logger(logging.Logger):
    def __init__(self, name):
        super().__init__(name, logging.INFO)
        h = logging.StreamHandler()
        h.setFormatter(logging.Formatter("    [%(name)s] %(levelname)s %(message)s"))
        self.addHandler(h)


class _Bridge:
    def __init__(self):
        self.handlers = {}
        self.target = None        # simuliertes MCU-Objekt (Test setzt es)
        self.calls = []
        self.lock = threading.Lock()

    def provide(self, name, fn):
        self.handlers[name] = fn

    def notify(self, name, *params):
        if name in self.handlers:
            self.handlers[name](*params)

    def call(self, name, *params, timeout=10):
        with self.lock:
            self.calls.append((name, params))
        if self.target is None:
            raise TimeoutError("kein MCU")
        fn = getattr(self.target, "rpc_" + name)
        return fn(*params)


Bridge = _Bridge()


class _App:
    def run(self, user_loop=None):
        raise RuntimeError("Im Test nicht verwendet")


App = _App()
