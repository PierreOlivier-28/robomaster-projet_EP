"""Approche, saisie, depose (Bilal). Ne pilote pas le robot : renvoie des commandes.

- Approcheur.step(objet, avant_cm) -> commande (vx, vy, vz) + indicateur "pret a saisir"
- saisir(arm) / deposer(arm) : sequences du bras (voir control/arm_ctrl.py)

Convention SDK : vz > 0 = rotation vers la droite (horaire).
"""
import config
from control.pid import PID


class Approcheur:
    def __init__(self):
        self.pid_rot = PID(config.KP_ROT, zone_morte=8.0, sortie_max=config.VITESSE_ROT_MAX)

    def reset(self):
        self.pid_rot.reset()

    def step(self, objet, avant_cm, dt=0.1):
        """objet = (cx, taille) en pixels (ou None si perdu). Retourne ((vx, vy, vz), pret)."""
        if objet is None:
            self.reset()
            return None, False
        cx, _taille = objet
        err_x = cx - config.CAMERA_LARGEUR_PX / 2.0            # > 0 : objet a droite
        vz = self.pid_rot.step(err_x, dt)
        cible, tol = config.DIST_PRISE_CM, config.TOL_DIST_PRISE_CM
        if abs(err_x) > 60:                                    # on s'oriente d'abord
            return (0.0, 0.0, vz), False
        ecart = avant_cm - cible                               # > 0 : trop loin
        if abs(err_x) <= 12 and abs(ecart) <= tol:
            return (0.0, 0.0, 0.0), True
        vx = max(-0.08, min(0.25, 0.006 * ecart))
        if 0 < abs(vx) < 0.04:
            vx = 0.04 if vx > 0 else -0.04
        return (vx, 0.0, vz), False


def saisir(arm):
    arm.prendre()
    return arm.objet_saisi()


def deposer(arm):
    arm.deposer()
