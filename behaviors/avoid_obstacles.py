"""Evitement reactif (Lenaic). Fonction PURE : renvoie une commande, ne pilote pas le robot."""
import config


def step(distance_cm):
    """Retourne (vx, vy, vz) si un obstacle est trop proche, sinon None."""
    if distance_cm < config.SEUIL_OBSTACLE_CM:
        return (0.0, 0.0, config.VITESSE_ROT_EVITEMENT)   # on s'arrete et on tourne
    return None
