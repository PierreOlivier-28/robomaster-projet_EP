"""Connexion UNIQUE au robot (reel ou simule). Tous les modules recoivent `ep`."""
from contextlib import contextmanager

import config


def connecter():
    """Retourne un objet `ep` (robot reel ou faux robot selon config.MODE)."""
    if config.MODE == "real":
        from robomaster import robot  # import tardif : inutile en simulation
        ep = robot.Robot()
        ep.initialize(conn_type=config.CONN_TYPE)
        return ep
    from sim.fake_robot import FakeRobot
    return FakeRobot()


def fermer(ep):
    """Arret propre : vitesse nulle puis fermeture."""
    try:
        ep.chassis.drive_speed(x=0, y=0, z=0, timeout=1)
    except Exception:
        pass
    ep.close()


@contextmanager
def robot_session():
    """with robot_session() as ep: ... -> fermeture garantie, meme en cas d'erreur."""
    ep = connecter()
    try:
        yield ep
    finally:
        fermer(ep)
