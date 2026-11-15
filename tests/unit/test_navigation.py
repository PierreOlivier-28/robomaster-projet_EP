import math

import config
from behaviors import avoid_obstacles
from behaviors.follow_person import Suiveur
from behaviors.navigate_a_to_b import Navigateur
from sim.fake_detector import FakeDetector
from sim.world2d import World

DT = 0.05


def naviguer(world, but=(3.0, 0.0), duree=60.0):
    nav = Navigateur(but)
    contacts = 0
    for _ in range(int(duree / DT)):
        tof = world.tof_cm()
        cmd, arrive = nav.step(world.odometrie(), tof[config.PORT_AVANT],
                               tof[config.PORT_GAUCHE], tof[config.PORT_DROITE], DT)
        if arrive:
            return True, contacts
        world.set_cmd(*cmd, timeout=0.5)
        world.step(DT)
        contacts += world.collision
    return False, contacts


def test_navigation_sans_obstacle():
    ok, contacts = naviguer(World())
    assert ok and contacts == 0


def test_navigation_terrain_de_demo_sans_collision():
    ok, contacts = naviguer(World.demo())
    assert ok
    assert contacts == 0


def test_navigation_obstacle_au_milieu():
    w = World()
    w.obstacles = [(1.8, 1.2, 2.2, 1.8)]
    ok, contacts = naviguer(w)
    assert ok and contacts < 10


def test_navigation_obstacle_que_le_capteur_avant_ne_voit_pas():
    # le rayon central passe a cote du coin, mais le corps du robot l'accroche :
    # le robot doit detecter qu'il n'avance plus et se decaler
    w = World()
    w.obstacles = [(1.1, 0.99, 1.45, 1.34)]
    ok, _ = naviguer(w)
    assert ok


def test_evitement_se_decale_du_cote_le_plus_libre():
    assert avoid_obstacles.step(80.0, 100.0, 100.0) is None
    droite = avoid_obstacles.step(30.0, 40.0, 150.0)     # plus de place a droite
    gauche = avoid_obstacles.step(30.0, 150.0, 40.0)     # plus de place a gauche
    assert droite[1] > 0 and gauche[1] < 0               # vy > 0 = vers la droite
    assert droite[0] == 0.0 and droite[2] == 0.0         # translation pure


def test_tracking_garde_la_personne_dans_le_champ():
    w = World()
    w.set_person(3.3, 0.5)
    assert w.placer_personne_devant()
    w.person_marche = True
    det, suiveur = FakeDetector(w), Suiveur()
    vue, total, dmin, dmax = 0, 0, 9.0, 0.0
    for i in range(int(60 / DT)):
        d = det.detect()
        cmd = suiveur.step(d[0][:4] if d else None, DT)
        w.set_cmd(*(cmd or (0.0, 0.0, 0.0)), timeout=0.5)
        w.step(DT)
        total += 1
        vue += bool(d)
        if i > 400:
            dist = math.hypot(w.person[0] - w.pose[0], w.person[1] - w.pose[1])
            dmin, dmax = min(dmin, dist), max(dmax, dist)
    assert vue / total > 0.9
    assert 0.6 < dmin and dmax < 2.2


def test_tracking_cible_perdue_arret_immediat():
    assert Suiveur().step(None) is None
