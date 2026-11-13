"""Securite : limites de vitesse, arret d'urgence, watchdog."""
import time

import config


def clamp(v, vmax):
    return max(-vmax, min(vmax, v))


def limiter_commande(vx, vy, vz):
    """Borne une commande (vx, vy en m/s ; vz en deg/s)."""
    return (clamp(vx, config.VITESSE_MAX),
            clamp(vy, config.VITESSE_MAX),
            clamp(vz, config.VITESSE_ROT_MAX))


def emergency_stop(ep):
    ep.chassis.drive_speed(x=0, y=0, z=0, timeout=1)


class Watchdog:
    """Declenche si on n'a pas 'nourri' le chien de garde depuis timeout_s."""

    def __init__(self, timeout_s=1.0):
        self.timeout_s = timeout_s
        self._last = time.time()

    def feed(self):
        self._last = time.time()

    def expired(self):
        return (time.time() - self._last) > self.timeout_s
