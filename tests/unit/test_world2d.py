import math

import config
from sim.fake_detector import FakeDetector
from sim.fake_robot import FakeRobot
from sim.world2d import World


def test_tof_avant_distance_a_un_obstacle():
    w = World()
    w.obstacles = [(2.0, 1.0, 2.3, 2.0)]          # le robot est en (0.5, 1.5), cap 0
    avant = w.tof_cm()[config.PORT_AVANT]
    attendu = (2.0 - 0.5 - config.ROBOT_RAYON_M) * 100
    assert abs(avant - attendu) < 0.5


def test_tof_rien_en_vue_donne_valeur_invalide():
    w = World(largeur=8.0, hauteur=8.0)
    assert w.tof_cm()[config.PORT_AVANT] >= config.DISTANCE_INVALIDE_CM


def test_la_commande_deplace_le_robot_et_expire():
    w = World()
    w.set_cmd(0.4, 0.0, 0.0, timeout=0.5)
    for _ in range(20):                           # 1 s : la commande expire apres 0,5 s
        w.step(0.05)
    x, y, yaw = w.odometrie()
    assert 0.15 < x < 0.25 and abs(y) < 1e-6      # ~0,4 m/s pendant 0,5 s = 0,2 m


def test_un_mur_bloque_le_robot():
    w = World()
    w.set_cmd(-0.5, 0.0, 0.0, timeout=5.0)        # recule vers le mur
    for _ in range(100):
        w.step(0.05)
    assert w.pose[0] >= config.ROBOT_RAYON_M - 1e-6
    assert w.collision


def test_convention_du_sdk_y_droite_et_z_horaire():
    w = World()
    w.set_cmd(0.0, 0.3, 0.0, timeout=5.0)         # y > 0 : vers la droite = bas de l'ecran
    for _ in range(10):
        w.step(0.05)
    assert w.odometrie()[1] > 0.1
    w2 = World()
    w2.set_cmd(0.0, 0.0, 30.0, timeout=5.0)       # z > 0 : rotation horaire
    for _ in range(20):
        w2.step(0.05)
    assert w2.odometrie()[2] > 20


def test_odometrie_et_changement_de_repere():
    w = World()
    w.pose = [1.0, 2.0, math.radians(90)]
    w.start = (0.5, 1.5, math.radians(90))
    x, y, yaw = w.odometrie()
    wx, wy = w.odom_vers_monde(x, y)
    assert abs(wx - 1.0) < 1e-9 and abs(wy - 2.0) < 1e-9
    assert abs(yaw) < 1e-9


def test_camera_voit_la_personne_devant_puis_plus_si_cachee():
    w = World()
    w.set_person(2.5, 1.5)
    boite = FakeDetector(w).detect()[0]
    cx = (boite[0] + boite[2]) / 2
    assert abs(cx - config.CAMERA_LARGEUR_PX / 2) < 1.0
    w.obstacles = [(1.4, 1.2, 1.7, 1.8)]          # un obstacle se met entre les deux
    assert FakeDetector(w).detect() == []


def test_faux_robot_relie_au_monde():
    w = World()
    ep = FakeRobot(world=w)
    ep.chassis.drive_speed(x=0.3, y=0, z=0, timeout=1.0)
    for _ in range(20):
        w.step(0.05)
    assert w.odometrie()[0] > 0.25
    assert len(w.tof_mm()) == 4
