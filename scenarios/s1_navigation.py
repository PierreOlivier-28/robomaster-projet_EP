"""Scenario 1 : avance + evitement d'obstacle. Tourne deja en simulation :
    python main.py s1
"""
import time

from behaviors import avoid_obstacles
from control.chassis_ctrl import Chassis
from core import logger
from core.robot_io import robot_session
from core.sensors import Distance
import config
from mission.arbiter import choisir


def main(duree_s=10.0):
    with robot_session() as ep:
        dist = Distance(ep)
        chassis = Chassis(ep)
        dist.start()
        t0 = time.time()
        try:
            while time.time() - t0 < duree_s:
                d = dist.front_cm()
                cmd = choisir([
                    avoid_obstacles.step(d),               # priorite 1 : evitement
                    (config.VITESSE_AVANT, 0.0, 0.0),      # sinon : on avance
                ])
                chassis.drive(*cmd)
                logger.log("front={:.0f} cm  cmd={}".format(d, cmd))
                time.sleep(0.2)
        finally:
            chassis.stop()
            dist.stop()
