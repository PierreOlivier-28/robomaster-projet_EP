"""Suivi de personne (Mohamed) : rotation pour centrer + avance/recul pour garder la distance.

step() ne pilote pas le robot : il renvoie une commande (vx, vy, vz), ou None si la cible est perdue.
"""
import config
from control.pid import PID


class Suiveur:
    def __init__(self):
        self.pid_rot = PID(config.KP_ROT, zone_morte=15.0, sortie_max=config.VITESSE_ROT_MAX)
        self.pid_dist = PID(config.KP_DIST, zone_morte=10.0, sortie_max=config.VITESSE_MAX)

    def reset(self):
        self.pid_rot.reset()
        self.pid_dist.reset()

    def step(self, boite, dt=0.1):
        """boite = (x1, y1, x2, y2, ...) de la personne dans l'image, ou None."""
        if boite is None:
            self.reset()
            return None
        x1, _y1, x2, _y2 = boite[:4]
        err_x = (x1 + x2) / 2.0 - config.CAMERA_LARGEUR_PX / 2.0     # > 0 : cible a droite
        err_d = config.LARGEUR_CIBLE_PX - (x2 - x1)                  # > 0 : cible trop loin
        vz = self.pid_rot.step(err_x, dt)
        vx = self.pid_dist.step(err_d, dt)
        return (vx, 0.0, vz)
