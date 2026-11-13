"""Detection YOLO (Mohamed). Le modele est charge UNE SEULE fois."""


class Detector:
    def __init__(self, modele="yolov8n.pt"):
        self.modele = modele
        self._model = None

    def charger(self):
        raise NotImplementedError("A ecrire : from ultralytics import YOLO ; YOLO(models/...)")

    def detect(self, frame):
        """Retourne une liste de boites (x1, y1, x2, y2, classe, confiance)."""
        raise NotImplementedError
