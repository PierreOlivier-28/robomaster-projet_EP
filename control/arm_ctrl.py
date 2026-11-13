"""Bras et pince (Bilal) : positions repos / prise / depose."""
import config


class Arm:
    def __init__(self, ep):
        self.ep = ep

    def repos(self):
        x, y = config.POS_BRAS["repos"]
        self.ep.robotic_arm.moveto(x=x, y=y)

    def prendre(self):
        raise NotImplementedError("A ecrire : approche, ouvrir, descendre, fermer, remonter")

    def deposer(self):
        raise NotImplementedError("A ecrire")
