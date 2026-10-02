# Test-Ersatz für den ObjectDetection-Brick: Ergebnis kommt aus einer Funktion des Tests.
HOOK = {"fn": None}


class ObjectDetection:
    def __init__(self, confidence=0.3):
        self.confidence = confidence

    def detect(self, image_bytes, image_type="jpg", confidence=None):
        fn = HOOK["fn"]
        dets = fn() if fn else []
        return {"detection": dets}
