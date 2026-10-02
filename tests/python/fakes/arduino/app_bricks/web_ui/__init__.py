# Test-Ersatz für den WebUI-Brick: merkt sich Nachrichten und Routen.
import threading


class WebUI:
    def __init__(self, *a, **k):
        self.handlers = {}
        self.routes = {}
        self.sent = []
        self.connect_cb = None
        self.lock = threading.Lock()

    def on_message(self, name, fn):
        self.handlers[name] = fn

    def on_connect(self, fn):
        self.connect_cb = fn

    def expose_api(self, method, path, fn):
        self.routes[(method, path)] = fn

    def send_message(self, name, message, room=None):
        with self.lock:
            self.sent.append((name, message))
            if len(self.sent) > 2000:
                del self.sent[:1000]

    def last(self, name):
        with self.lock:
            for n, m in reversed(self.sent):
                if n == name:
                    return m
        return None

    def emit(self, name, data=None):
        return self.handlers[name]("sid-test", data or {})
