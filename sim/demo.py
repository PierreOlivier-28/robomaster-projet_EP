"""Simulateur interactif 2D :  python main.py sim

Les comportements utilises sont EXACTEMENT ceux du projet (behaviors/*), branches
sur un faux robot (FakeRobot) relie au monde 2D. Rien n'est specifique a la simulation.

Touches : 1 evitement A->B | 2 tracking | 3 prise d'objet
          4 mission complete (prendre, aller en B, deposer) | m manuel
          r reset | espace pause | p personne marche on/off | ESC ou x quitter
Souris  : clic gauche = placer le but B (ou saisir/deplacer la personne,
          ou deplacer l'objet en modes 4 et 5)
          clic droit  = ajouter / retirer un obstacle
"""
import math
import time

import config
from behaviors.follow_person import Suiveur
from behaviors.navigate_a_to_b import Navigateur
from control.arm_ctrl import Arm
from control.chassis_ctrl import Chassis
from core.odometry import Odometrie
from core.sensors import Distance
from mission.arbiter import ARRET, choisir
from mission.state_machine import Capteurs, Mission
from sim.fake_detector import FakeDetector
from sim.fake_object_locator import FakeObjectLocator
from sim.fake_robot import FakeRobot
from sim.viewer import Viewer
from sim.world2d import World

NOMS = {
    "m": "MANUEL (w a s d q e)",
    "1": "1 EVITEMENT A->B",
    "2": "2 TRACKING",
    "3": "3 PRISE D'OBJET",
    "4": "4 MISSION COMPLETE",
}
AIDE = [
    "1 evitement A->B (decalage lateral)",
    "2 tracking  3 prise d'objet",
    "4 mission complete",
    "m manuel (w a s d, q e)",
    "clic gauche : changer B / deplacer personne",
    "clic droit  : ajouter/retirer obstacle",
    "p personne marche   r reset",
    "espace pause   ESC ou x quitter",
]


def main():
    world = World.demo()
    ep = FakeRobot(world=world)
    chassis = Chassis(ep)
    dist = Distance(ep, freq=20)
    odo = Odometrie(ep, freq=20)
    detecteur = FakeDetector(world)
    localisateur = FakeObjectLocator(world)
    bras = Arm(ep)
    mission = None
    BUT_B = (3.0, 0.0)                  # but de la mission, en coordonnees d'odometrie
    suiveur = Suiveur()
    dist.start()
    odo.start()
    viewer = Viewer(world)

    mode = "1"
    pause = False
    but_odom = BUT_B                    # A = depart du robot, B = arrivee (clic = changer B)
    navigateur = Navigateur(but_odom)
    glisse_personne = False
    obstacles_sauves = None
    t_prec = time.time()

    try:
        while True:
            maintenant = time.time()
            dt = min(0.05, maintenant - t_prec)
            t_prec = maintenant

            # ------------------------------------------------ capteurs (comme sur le robot)
            avant, gauche, droite = dist.front_cm(), dist.left_cm(), dist.right_cm()
            pose = odo.pose()

            # ------------------------------------------------ souris
            for typ, x, y in viewer.prendre_evenements():
                if typ == "gauche":
                    if world.person is not None and math.hypot(
                            x - world.person[0], y - world.person[1]) < 0.3:
                        glisse_personne = True
                        world.person_marche = False
                    elif mode == "1":
                        but_odom = world.monde_vers_odom(x, y)
                        navigateur = Navigateur(but_odom)
                    elif mode in ("3", "4") and not world.objet_tenu:
                        world.reset()                         # on repart du depart
                        world.set_objet(x, y)
                        world.activer_objet(True)
                        mission = Mission(bras, but=None if mode == "3" else BUT_B)
                elif typ == "glisse" and glisse_personne:
                    world.person = [x, y]
                elif typ == "relache":
                    glisse_personne = False
                elif typ == "droit":
                    if not world.retirer_obstacle_en(x, y):
                        world.ajouter_obstacle(x, y)

            # ------------------------------------------------ decision
            cmd, etat = ARRET, ""
            if mode == "1":
                cmd, arrive = navigateur.step(pose, avant, gauche, droite, dt)
                if arrive:
                    etat = "ARRIVE en B !"
                elif navigateur.etat in ("STRAFE", "MARGE"):
                    etat = "obstacle : decalage vers la {}".format(
                        "droite" if navigateur.cote == 1 else "gauche")
                elif navigateur.etat == "AVANCE":
                    etat = "je longe l'obstacle"
                else:
                    etat = "cap sur B"
            elif mode in ("3", "4"):
                if mission is None:
                    world.reset()
                    world.activer_objet(True)
                    mission = Mission(bras, but=None if mode == "3" else BUT_B)
                cmd = mission.step(Capteurs(avant, gauche, droite, pose,
                                            localisateur.locate()), dt)
                etat = mission.etat + (" - " + mission.message if mission.message else "")
                if mission.etat == "FIN" and world.objet_dans_zone():
                    etat = "FIN - objet depose dans la zone !"
            elif mode == "2":
                det = detecteur.detect()
                suivi = suiveur.step(det[0][:4] if det else None, dt)
                etat = "cible suivie" if suivi is not None else "cible perdue : arret"
                cmd = choisir([ARRET if avant < 25 else None, suivi])

            # ------------------------------------------------ clavier
            info = {
                "mode": NOMS[mode], "cmd": cmd, "etat": etat, "aide": AIDE,
                "tof": world.tof_cm(), "pose_odom": pose,
                "but": world.odom_vers_monde(*but_odom) if (mode == "1" and but_odom) else None,
            }
            touche = viewer.afficher(viewer.render(info), 15)
            if touche in (27, ord("x")):
                break
            elif touche == ord(" "):
                pause = not pause
            elif touche == ord("r"):
                world.reset()
                navigateur, mission = Navigateur(but_odom), None
                suiveur.reset()
            elif touche == ord("p"):
                world.person_marche = not world.person_marche
            elif touche in (ord("1"), ord("2"), ord("3"), ord("4"), ord("m")):
                nouveau = chr(touche)
                if nouveau != mode or nouveau in ("3", "4"):
                    if mode == "2" and obstacles_sauves is not None:
                        world.obstacles = obstacles_sauves       # on remet les obstacles
                        obstacles_sauves = None
                    mode = nouveau
                    mission = None
                    navigateur = None
                    world.activer_objet(False)
                    if mode == "1":
                        world.reset()
                        navigateur = Navigateur(but_odom)
                    elif mode in ("3", "4"):
                        world.reset()
                        world.activer_objet(True)
                        mission = Mission(bras, but=None if mode == "3" else BUT_B)
                    elif mode == "2":
                        # le suivi se teste sans obstacle (comme dans le plan de tests)
                        obstacles_sauves = list(world.obstacles)
                        world.obstacles = []
                        suiveur.reset()
                        world.placer_personne_devant()
                        world.person_marche = True
            if mode == "m":
                vitesses = {ord("w"): (0.3, 0, 0), ord("s"): (-0.3, 0, 0),
                            ord("a"): (0, -0.3, 0), ord("d"): (0, 0.3, 0),
                            ord("q"): (0, 0, -45), ord("e"): (0, 0, 45)}
                cmd = vitesses.get(touche, ARRET)

            # ------------------------------------------------ action + physique
            chassis.drive(*cmd)
            if not pause:
                world.step(dt)
    finally:
        chassis.stop()
        dist.stop()
        odo.stop()
        viewer.fermer()


if __name__ == "__main__":
    main()
