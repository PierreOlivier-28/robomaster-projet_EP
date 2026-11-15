"""Detecteur simule : meme format de sortie que perception/detector.py (Detector.detect).

Permet de tester le tracking et l'asservissement SANS YOLO, dans le simulateur 2D.
"""


class FakeDetector:
    def __init__(self, world):
        self.world = world

    def detect(self, frame=None):
        """Liste de boites (x1, y1, x2, y2, classe, confiance) ; vide si personne n'est visible."""
        b = self.world.boite_personne()
        if b is None:
            return []
        return [(b[0], b[1], b[2], b[3], "person", 0.9)]
