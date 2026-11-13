"""Tracking avec ID et verrouillage de la cible (Mohamed)."""


class Tracker:
    def update(self, frame):
        """Retourne (cible, erreur_x, erreur_taille) ou (None, 0, 0) si cible perdue."""
        raise NotImplementedError
