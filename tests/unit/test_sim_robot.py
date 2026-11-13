import time

from control.chassis_ctrl import Chassis
from core.sensors import Distance
from sim import fake_tof
from sim.fake_robot import FakeRobot


def test_chassis_borne_la_vitesse():
    ep = FakeRobot()
    Chassis(ep).drive(10, 0, 0)
    assert ep.chassis.commandes[-1][0] <= 0.5


def test_distance_lue_depuis_le_faux_robot():
    ep = FakeRobot(fonction_distance=fake_tof.constante(42.0))
    dist = Distance(ep, freq=50)
    dist.start()
    time.sleep(0.2)
    dist.stop()
    assert abs(dist.front_cm() - 42.0) < 0.5
