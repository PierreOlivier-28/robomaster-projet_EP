"""Flux video : toujours lire la DERNIERE image (evite la latence)."""


class Camera:
    def __init__(self, ep, largeur=640):
        self.ep = ep
        self.largeur = largeur

    def start(self):
        self.ep.camera.start_video_stream(display=False)

    def read(self):
        """Derniere image (tableau numpy BGR), redimensionnee si besoin."""
        frame = self.ep.camera.read_cv2_image(strategy="newest", timeout=1)
        if frame is not None and self.largeur and frame.shape[1] != self.largeur:
            import cv2
            h = int(frame.shape[0] * self.largeur / frame.shape[1])
            frame = cv2.resize(frame, (self.largeur, h))
        return frame

    def stop(self):
        try:
            self.ep.camera.stop_video_stream()
        except Exception:
            pass
