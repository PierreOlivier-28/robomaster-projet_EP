"""Camera simulee : webcam du PC (0) ou video enregistree au labo (chemin .mp4)."""


class FakeCamera:
    def __init__(self, source=0):
        self.source = source
        self._cap = None

    def start_video_stream(self, display=False):
        import cv2
        self._cap = cv2.VideoCapture(self.source)

    def read_cv2_image(self, strategy="newest", timeout=1):
        if self._cap is None:
            return None
        ok, frame = self._cap.read()
        if not ok and isinstance(self.source, str):   # fin de video : on reboucle
            self._cap.set(1, 0)
            ok, frame = self._cap.read()
        return frame if ok else None

    def stop_video_stream(self):
        if self._cap is not None:
            self._cap.release()
            self._cap = None
