"""Commandes de deplacement bornees, avec timeout."""
import config
from core.safety import limiter_commande


class Chassis:
    def __init__(self, ep):
        self.ep = ep

    def drive(self, vx=0.0, vy=0.0, vz=0.0):
        vx, vy, vz = limiter_commande(vx, vy, vz)
        self.ep.chassis.drive_speed(x=vx, y=vy, z=vz, timeout=config.CMD_TIMEOUT_S)

    def stop(self):
        self.ep.chassis.drive_speed(x=0, y=0, z=0, timeout=1)
