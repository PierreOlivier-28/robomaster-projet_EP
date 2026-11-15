import math

import config
from behaviors.pick_place import Approcheur
from control.arm_ctrl import Arm
from sim.fake_object_locator import FakeObjectLocator
from sim.fake_robot import FakeRobot
from sim.world2d import World


def monde_avec_objet_devant(distance_bord_cm):
    """Robot en (0.5, 1.5) cap 0 ; objet pose devant, a `distance_bord_cm` du bord du robot."""
    w = World.demo()
    w.obstacles = []
    centre = config.ROBOT_RAYON_M + distance_bord_cm / 100.0 + config.OBJET_RAYON_M
    w.set_objet(0.5 + centre, 1.5)
    w.activer_objet(True)
    return w


def test_prise_reussie_a_la_bonne_distance():
    w = monde_avec_objet_devant(config.DIST_PRISE_CM)
    arm = Arm(FakeRobot(world=w))
    arm.prendre()
    assert w.objet_tenu and arm.objet_saisi()


def test_prise_ratee_si_objet_trop_loin():
    w = monde_avec_objet_devant(45.0)
    arm = Arm(FakeRobot(world=w))
    arm.prendre()
    assert not w.objet_tenu


def test_prise_ratee_si_le_bras_nest_pas_en_position_de_prise():
    w = monde_avec_objet_devant(config.DIST_PRISE_CM)
    ep = FakeRobot(world=w)
    ep.robotic_arm.moveto(x=0, y=0)               # mauvaise position
    ep.gripper.close()
    assert not w.objet_tenu


def test_prise_ratee_si_objet_decale_sur_le_cote():
    w = monde_avec_objet_devant(config.DIST_PRISE_CM)
    w.objet[1] += 0.12                            # ~15 degres a cote : hors de la pince
    Arm(FakeRobot(world=w)).prendre()
    assert not w.objet_tenu


def test_objet_tenu_suit_le_robot_puis_depose_dans_la_zone():
    w = monde_avec_objet_devant(config.DIST_PRISE_CM)
    arm = Arm(FakeRobot(world=w))
    arm.prendre()
    w.pose = [3.4, 1.5, 0.0]                      # le robot arrive dans la zone de depose
    w.step(0.05)
    assert w.objet_tenu and abs(w.objet[0] - (3.4 + config.ROBOT_RAYON_M + 0.10)) < 1e-6
    arm.deposer()
    assert not w.objet_tenu and w.objet_dans_zone()


def test_localisateur_voit_l_objet_au_centre_de_l_image():
    w = monde_avec_objet_devant(60.0)
    cx, taille = FakeObjectLocator(w).locate()
    assert abs(cx - config.CAMERA_LARGEUR_PX / 2) < 1.0 and taille > 5
    w.obstacles = [(0.8, 1.3, 1.0, 1.7)]          # un obstacle le cache
    assert FakeObjectLocator(w).locate() is None


def test_approche_s_arrete_a_la_bonne_distance_et_aligne():
    w = monde_avec_objet_devant(120.0)
    w.pose[2] = math.radians(10)                  # le robot n'est pas bien aligne
    loc, appro = FakeObjectLocator(w), Approcheur()
    pret = False
    for _ in range(int(30 / 0.05)):
        t = w.tof_cm()
        cmd, pret = appro.step(loc.locate(), t[config.PORT_AVANT], 0.05)
        if pret:
            break
        w.set_cmd(*(cmd or (0, 0, 0)), timeout=0.5)
        w.step(0.05)
    assert pret
    assert abs(w.distance_objet_cm() - config.DIST_PRISE_CM) <= config.TOL_DIST_PRISE_CM
    Arm(FakeRobot(world=w)).prendre()
    assert w.objet_tenu
