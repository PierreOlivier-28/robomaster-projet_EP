"""Bras et pince (Bilal) : positions repos / prise / depose (config.POS_BRAS, en mm).

Marche avec le vrai robot ET avec le faux robot du simulateur.
"""
import time

import config


class Arm:
    def __init__(self, ep):
        self.ep = ep

    def _attendre(self, action=None):
        if action is not None and hasattr(action, "wait_for_completed"):
            action.wait_for_completed()
        if config.MODE == "real":
            time.sleep(1.0)          # la pince n'a pas d'action a attendre

    def aller(self, nom):
        """Deplace le bras vers config.POS_BRAS[nom] ('repos', 'prise' ou 'depose')."""
        x, y = config.POS_BRAS[nom]
        self._attendre(self.ep.robotic_arm.moveto(x=x, y=y))

    def repos(self):
        self.aller("repos")

    def ouvrir(self):
        self._attendre(self.ep.gripper.open(power=config.PUISSANCE_PINCE))

    def fermer(self):
        self._attendre(self.ep.gripper.close(power=config.PUISSANCE_PINCE))

    def prendre(self):
        """Ouvrir la pince, aller en position de prise, fermer, remonter au repos."""
        self.ouvrir()
        self.aller("prise")
        self.fermer()
        self.aller("repos")

    def deposer(self):
        """Aller en position de depose, ouvrir, revenir au repos."""
        self.aller("depose")
        self.ouvrir()
        self.aller("repos")

    def objet_saisi(self):
        """True si la pince tient l'objet. Dans le simulateur : info exacte.
        Sur le vrai robot : A IMPLEMENTER avec l'etat de la pince (gripper.sub_status)."""
        verif = getattr(self.ep.gripper, "objet_saisi", None)
        return verif() if verif is not None else True
