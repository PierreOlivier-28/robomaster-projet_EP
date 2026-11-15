import config
from control.arm_ctrl import Arm
from mission.state_machine import Capteurs, Mission
from sim.fake_object_locator import FakeObjectLocator
from sim.fake_robot import FakeRobot
from sim.world2d import World

DT = 0.05
BUT_B = (3.0, 0.0)


def jouer(world, but, duree=150.0):
    world.activer_objet(True)
    arm, loc = Arm(FakeRobot(world=world)), FakeObjectLocator(world)
    mission = Mission(arm, but=but)
    etats, contacts = [], 0
    for _ in range(int(duree / DT)):
        t = world.tof_cm()
        c = Capteurs(t[config.PORT_AVANT], t[config.PORT_GAUCHE], t[config.PORT_DROITE],
                     world.odometrie(), loc.locate())
        cmd = mission.step(c, DT)
        world.set_cmd(*cmd, timeout=0.5)
        world.step(DT)
        contacts += world.collision
        if not etats or etats[-1] != mission.etat:
            etats.append(mission.etat)
        if mission.terminee:
            break
    return mission, etats, contacts


def test_mission_complete_prendre_aller_en_b_deposer():
    w = World.demo()
    mission, etats, contacts = jouer(w, BUT_B)
    assert mission.etat == "FIN"
    assert etats == ["INIT", "CHERCHER", "APPROCHER", "SAISIR", "NAVIGUER", "DEPOSER", "FIN"] \
        or etats == ["CHERCHER", "APPROCHER", "SAISIR", "NAVIGUER", "DEPOSER", "FIN"]
    assert w.objet_dans_zone()
    assert w.t < 120.0
    assert contacts < 60


def test_mission_prise_seule_depose_sur_place():
    w = World.demo()
    mission, _, _ = jouer(w, None)
    assert mission.etat == "FIN"
    assert not w.objet_tenu and not w.objet_dans_zone()


def test_mission_objet_ailleurs():
    w = World.demo()
    w.set_objet(0.9, 2.0)                         # plus bas que dans la demo
    mission, _, _ = jouer(w, BUT_B)
    assert mission.etat == "FIN" and w.objet_dans_zone()


def test_mission_erreur_si_objet_introuvable():
    w = World.demo()
    w.obstacles = [(0.0, 0.0, 0.01, 0.01)]
    arm, mission = Arm(FakeRobot(world=w)), None
    w.activer_objet(False)                        # l'objet n'existe pas
    mission = Mission(arm, but=BUT_B)
    for _ in range(int(40 / DT)):
        t = w.tof_cm()
        c = Capteurs(t[0], t[3], t[1], w.odometrie(), FakeObjectLocator(w).locate())
        w.set_cmd(*mission.step(c, DT), timeout=0.5)
        w.step(DT)
        if mission.terminee:
            break
    assert mission.etat == "ERREUR" and "introuvable" in mission.message
