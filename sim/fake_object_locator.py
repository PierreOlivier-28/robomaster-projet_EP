"""Localisation d'objet simulee : meme sortie que perception/object_locator.py.

locate(frame) -> (cx, taille) en pixels (centre horizontal et largeur de l'objet), ou None.
Permet de developper la prise d'objet SANS vision, dans le simulateur 2D.
"""


class FakeObjectLocator:
    def __init__(self, world):
        self.world = world

    def locate(self, frame=None):
        b = self.world.boite_objet()
        if b is None:
            return None
        return ((b[0] + b[2]) / 2.0, b[2] - b[0])
