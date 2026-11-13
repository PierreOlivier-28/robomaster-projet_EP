#!/usr/bin/env python
"""Cree le squelette du projet RoboMaster dans le dossier courant.

Usage (a la racine du depot clone) :
    python setup_skeleton.py

- Ne remplace JAMAIS un fichier qui existe deja (il est signale "existe").
- Compatible Python 3.8+.
"""
import os
import sys

FILES = {}

# --------------------------------------------------------------------------- racine
FILES["README.md"] = r'''# Projet DJI RoboMaster EP - ESIGELEC 2026/2027

Robotique mobile et perception : navigation/evitement (S1), tracking de personne (S2),
manipulation d'objet (S3), mission complete.

## Installation (Windows, Python 3.8 OBLIGATOIRE)
Le SDK `robomaster` ne s'installe pas sous Python 3.9+.

```powershell
py -3.8 --version                 # doit afficher 3.8.x
py -3.8 -m venv venv
venv\Scripts\activate
python -m pip install --upgrade pip
pip install -r requirements.txt
```

Faire TOUTES les installations (pip, modeles YOLO) AVANT de se connecter au Wi-Fi du robot :
le mode AP n'a pas d'acces Internet.

## Lancer (mode simulation, sans robot)
```powershell
python main.py s1          # navigation + evitement (faux robot)
python main.py s2
python main.py s3
python main.py mission
pytest                     # tests sans robot
```

## Lancer avec le vrai robot (au labo)
PC connecte au Wi-Fi RMEP-xxxxxx, robot allume :
```powershell
python main.py s1 --real
pytest -m robot            # tests qui necessitent le robot
```

## Organisation
Voir `docs/` et l'onglet "Architecture code" du fichier de gestion de projet.
Dependances vers le bas uniquement :
`scenarios -> mission -> behaviors -> perception / control -> core`.

## Regles d'equipe
- Jamais de push direct sur `main` : branche + pull request + 1 relecture.
- Chacun ne modifie que ses fichiers (voir `.github/CODEOWNERS`).
- `pytest` doit passer avant chaque PR.
- Tout test sur robot est note dans le Plan de tests ; tag Git apres chaque seance robot.
- IA utilisee : l'ecrire dans le Registre IA.
'''

FILES["requirements.txt"] = r'''# Python 3.8 obligatoire. Une fois tout installe et fonctionnel : pip freeze > requirements.txt
robomaster
opencv-python
numpy
pytest>=7
# YOLO (Mohamed, a installer AVANT la connexion au robot) :
# torch
# ultralytics
'''

FILES["pytest.ini"] = r'''[pytest]
pythonpath = .
testpaths = tests
markers =
    robot: test qui necessite le vrai robot (au labo : pytest -m robot)
addopts = -m "not robot"
'''

FILES["config.py"] = r'''"""Tous les reglages du projet. Aucun nombre magique dans le reste du code."""
import os

# --- Mode : "sim" (faux robot, chez soi) ou "real" (vrai robot, au labo)
MODE = os.environ.get("ROBOT_MODE", "sim")

# --- Connexion
CONN_TYPE = "ap"            # mode point d'acces : PC sur le Wi-Fi RMEP-xxxxxx

# --- Securite
CMD_TIMEOUT_S = 0.5         # chaque commande de vitesse expire apres ce delai
VITESSE_MAX = 0.5           # m/s (avant/lateral)
VITESSE_ROT_MAX = 60.0      # deg/s

# --- Capteurs ToF
PORT_AVANT = 0              # index du capteur ToF avant (a verifier sur le robot)
SEUIL_OBSTACLE_CM = 50
DISTANCE_INVALIDE_CM = 999.0

# --- Mouvements
VITESSE_AVANT = 0.3
VITESSE_ROT_EVITEMENT = 40.0

# --- Asservissement (a regler en S2 / S3)
KP_ROT = 0.1
KP_DIST = 0.002

# --- Bras / pince (a calibrer avec tools/calibration_bras.py)
POS_BRAS = {
    "repos": (0, 0),
    "prise": (0, 0),
    "depose": (0, 0),
}

# --- Chemins
LOG_DIR = "logs"
MODEL_DIR = "models"
DATA_DIR = "data"
'''

FILES["main.py"] = r'''"""Point d'entree : python main.py s1|s2|s3|mission [--real]"""
import argparse
import os


def main(argv=None):
    parser = argparse.ArgumentParser(description="Projet RoboMaster")
    parser.add_argument("scenario", choices=["s1", "s2", "s3", "mission"])
    parser.add_argument("--real", action="store_true",
                        help="utiliser le vrai robot (sinon : simulation)")
    args = parser.parse_args(argv)

    # le mode doit etre fixe AVANT d'importer config / les scenarios
    os.environ["ROBOT_MODE"] = "real" if args.real else "sim"

    if args.scenario == "s1":
        from scenarios import s1_navigation as sc
    elif args.scenario == "s2":
        from scenarios import s2_tracking as sc
    elif args.scenario == "s3":
        from scenarios import s3_manipulation as sc
    else:
        from scenarios import s4_mission_complete as sc
    sc.main()


if __name__ == "__main__":
    main()
'''

# --------------------------------------------------------------------------- core
FILES["core/__init__.py"] = ""

FILES["core/robot_io.py"] = r'''"""Connexion UNIQUE au robot (reel ou simule). Tous les modules recoivent `ep`."""
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
'''

FILES["core/sensors.py"] = r'''"""Lecture des capteurs ToF (distances en cm, valeurs invalides filtrees)."""
import config


class Distance:
    def __init__(self, ep, freq=10):
        self.ep = ep
        self.freq = freq
        self._mm = []

    def _callback(self, sub_info):
        self._mm = list(sub_info)

    def start(self):
        self.ep.sensor.sub_distance(freq=self.freq, callback=self._callback)

    def stop(self):
        try:
            self.ep.sensor.unsub_distance()
        except Exception:
            pass

    @staticmethod
    def _valide(mm):
        return mm is not None and 0 < mm < 3000

    def front_cm(self):
        """Distance du capteur avant en cm (config.DISTANCE_INVALIDE_CM si inconnue)."""
        if len(self._mm) <= config.PORT_AVANT or not self._valide(self._mm[config.PORT_AVANT]):
            return config.DISTANCE_INVALIDE_CM
        return self._mm[config.PORT_AVANT] / 10.0

    def min_cm(self):
        """Plus petite distance valide parmi les 4 capteurs."""
        valides = [m / 10.0 for m in self._mm if self._valide(m)]
        return min(valides) if valides else config.DISTANCE_INVALIDE_CM
'''

FILES["core/camera.py"] = r'''"""Flux video : toujours lire la DERNIERE image (evite la latence)."""


class Camera:
    def __init__(self, ep, largeur=640):
        self.ep = ep
        self.largeur = largeur

    def start(self):
        self.ep.camera.start_video_stream(display=False)

    def read(self):
        """Derniere image (tableau numpy BGR), redimensionnee si besoin."""
        frame = self.ep.camera.read_cv2_image(strategy="newest", timeout=1)
        if frame is not None and self.largeur and frame.shape[1] != self.largeur:
            import cv2
            h = int(frame.shape[0] * self.largeur / frame.shape[1])
            frame = cv2.resize(frame, (self.largeur, h))
        return frame

    def stop(self):
        try:
            self.ep.camera.stop_video_stream()
        except Exception:
            pass
'''

FILES["core/safety.py"] = r'''"""Securite : limites de vitesse, arret d'urgence, watchdog."""
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
'''

FILES["core/logger.py"] = r'''"""Journal des essais et sauvegarde d'images/videos pour rejouer chez soi."""
import os
import time

import config

_fichier = None


def _chemin():
    global _fichier
    if _fichier is None:
        os.makedirs(config.LOG_DIR, exist_ok=True)
        _fichier = os.path.join(config.LOG_DIR, time.strftime("essai_%Y%m%d_%H%M%S.log"))
    return _fichier


def log(msg):
    ligne = "[{}] {}".format(time.strftime("%H:%M:%S"), msg)
    print(ligne)
    with open(_chemin(), "a", encoding="utf-8") as f:
        f.write(ligne + "\n")


def save_frame(frame, nom=None):
    import cv2
    os.makedirs(config.LOG_DIR, exist_ok=True)
    nom = nom or time.strftime("frame_%Y%m%d_%H%M%S_%f.jpg")
    cv2.imwrite(os.path.join(config.LOG_DIR, nom), frame)
'''

# --------------------------------------------------------------------------- sim
FILES["sim/__init__.py"] = ""

FILES["sim/fake_tof.py"] = r'''"""Distances ToF simulees : fonctions f(t_secondes) -> cm."""
import csv


def constante(cm):
    return lambda t: cm


def onde(min_cm=20.0, max_cm=150.0, periode_s=8.0):
    """Le robot 'approche' puis 'recule' d'un obstacle (triangle)."""
    def f(t):
        phase = (t % periode_s) / periode_s
        tri = 1 - abs(2 * phase - 1)          # 0 -> 1 -> 0
        return max_cm - tri * (max_cm - min_cm)
    return f


def rejouer_csv(chemin, colonne="front_cm"):
    """Rejoue un enregistrement fait au labo (colonnes : t, front_cm)."""
    with open(chemin, newline="") as fh:
        lignes = [(float(r["t"]), float(r[colonne])) for r in csv.DictReader(fh)]

    def f(t):
        valeur = lignes[0][1]
        for ti, vi in lignes:
            if ti > t:
                break
            valeur = vi
        return valeur
    return f
'''

FILES["sim/fake_camera.py"] = r'''"""Camera simulee : webcam du PC (0) ou video enregistree au labo (chemin .mp4)."""


class FakeCamera:
    def __init__(self, source=0):
        self.source = source
        self._cap = None

    def start_video_stream(self, display=False):
        import cv2
        self._cap = cv2.VideoCapture(self.source)

    def read_cv2_image(self, strategy="newest", timeout=1):
        if self._cap is None:
            return None
        ok, frame = self._cap.read()
        if not ok and isinstance(self.source, str):   # fin de video : on reboucle
            self._cap.set(1, 0)
            ok, frame = self._cap.read()
        return frame if ok else None

    def stop_video_stream(self):
        if self._cap is not None:
            self._cap.release()
            self._cap = None
'''

FILES["sim/fake_robot.py"] = r'''"""Faux robot : meme interface que le SDK robomaster (parties utilisees par le projet)."""
import threading
import time

import config
from sim import fake_tof
from sim.fake_camera import FakeCamera


class FakeChassis:
    def __init__(self):
        self.commandes = []          # historique pour les tests

    def drive_speed(self, x=0, y=0, z=0, timeout=None):
        self.commandes.append((x, y, z))


class FakeSensor:
    def __init__(self, fonction_distance=None):
        self.fonction = fonction_distance or fake_tof.onde()
        self._actif = False

    def sub_distance(self, freq=10, callback=None):
        self._actif = True
        t0 = time.time()

        def boucle():
            while self._actif:
                cm = self.fonction(time.time() - t0)
                mm = [3000.0] * 4
                mm[config.PORT_AVANT] = cm * 10.0
                callback(mm)
                time.sleep(1.0 / freq)

        threading.Thread(target=boucle, daemon=True).start()

    def unsub_distance(self):
        self._actif = False


class FakeArm:
    def moveto(self, x=0, y=0):
        print("[sim] bras -> ({}, {})".format(x, y))


class FakeGripper:
    def open(self, power=50):
        print("[sim] pince ouverte")

    def close(self, power=50):
        print("[sim] pince fermee")


class FakeRobot:
    def __init__(self, fonction_distance=None, source_camera=0):
        self.chassis = FakeChassis()
        self.sensor = FakeSensor(fonction_distance)
        self.camera = FakeCamera(source_camera)
        self.robotic_arm = FakeArm()
        self.gripper = FakeGripper()

    def close(self):
        self.sensor.unsub_distance()
        self.camera.stop_video_stream()
'''

# --------------------------------------------------------------------------- control
FILES["control/__init__.py"] = ""

FILES["control/pid.py"] = r'''"""Regulateur PID generique avec zone morte et saturation."""


class PID:
    def __init__(self, kp, ki=0.0, kd=0.0, zone_morte=0.0, sortie_max=None):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.zone_morte = zone_morte
        self.sortie_max = sortie_max
        self._integrale = 0.0
        self._prec = 0.0

    def reset(self):
        self._integrale = 0.0
        self._prec = 0.0

    def step(self, erreur, dt=0.1):
        if abs(erreur) < self.zone_morte:
            erreur = 0.0
        self._integrale += erreur * dt
        derivee = (erreur - self._prec) / dt if dt > 0 else 0.0
        self._prec = erreur
        sortie = self.kp * erreur + self.ki * self._integrale + self.kd * derivee
        if self.sortie_max is not None:
            sortie = max(-self.sortie_max, min(self.sortie_max, sortie))
        return sortie
'''

FILES["control/chassis_ctrl.py"] = r'''"""Commandes de deplacement bornees, avec timeout."""
import config
from core.safety import limiter_commande


class Chassis:
    def __init__(self, ep):
        self.ep = ep

    def drive(self, vx=0.0, vy=0.0, vz=0.0):
        vx, vy, vz = limiter_commande(vx, vy, vz)
        self.ep.chassis.drive_speed(x=vx, y=vy, z=vz, timeout=config.CMD_TIMEOUT_S)

    def stop(self):
        self.ep.chassis.drive_speed(x=0, y=0, z=0, timeout=1)
'''

FILES["control/arm_ctrl.py"] = r'''"""Bras et pince (Bilal) : positions repos / prise / depose."""
import config


class Arm:
    def __init__(self, ep):
        self.ep = ep

    def repos(self):
        x, y = config.POS_BRAS["repos"]
        self.ep.robotic_arm.moveto(x=x, y=y)

    def prendre(self):
        raise NotImplementedError("A ecrire : approche, ouvrir, descendre, fermer, remonter")

    def deposer(self):
        raise NotImplementedError("A ecrire")
'''

# --------------------------------------------------------------------------- perception
FILES["perception/__init__.py"] = ""

FILES["perception/detector.py"] = r'''"""Detection YOLO (Mohamed). Le modele est charge UNE SEULE fois."""


class Detector:
    def __init__(self, modele="yolov8n.pt"):
        self.modele = modele
        self._model = None

    def charger(self):
        raise NotImplementedError("A ecrire : from ultralytics import YOLO ; YOLO(models/...)")

    def detect(self, frame):
        """Retourne une liste de boites (x1, y1, x2, y2, classe, confiance)."""
        raise NotImplementedError
'''

FILES["perception/tracker.py"] = r'''"""Tracking avec ID et verrouillage de la cible (Mohamed)."""


class Tracker:
    def update(self, frame):
        """Retourne (cible, erreur_x, erreur_taille) ou (None, 0, 0) si cible perdue."""
        raise NotImplementedError
'''

FILES["perception/object_locator.py"] = r'''"""Position de l'objet a saisir dans l'image (Bilal)."""


def locate(frame):
    """Retourne (cx, taille) de l'objet dans l'image, ou None."""
    raise NotImplementedError
'''

# --------------------------------------------------------------------------- behaviors
FILES["behaviors/__init__.py"] = ""

FILES["behaviors/avoid_obstacles.py"] = r'''"""Evitement reactif (Lenaic). Fonction PURE : renvoie une commande, ne pilote pas le robot."""
import config


def step(distance_cm):
    """Retourne (vx, vy, vz) si un obstacle est trop proche, sinon None."""
    if distance_cm < config.SEUIL_OBSTACLE_CM:
        return (0.0, 0.0, config.VITESSE_ROT_EVITEMENT)   # on s'arrete et on tourne
    return None
'''

FILES["behaviors/navigate_a_to_b.py"] = r'''"""Navigation A -> B (Lenaic)."""


def step(pose):
    """Retourne (commande, arrive)."""
    raise NotImplementedError
'''

FILES["behaviors/follow_person.py"] = r'''"""Suivi de personne (Mohamed) : rotation + avance/recul."""


def step(cible):
    """Retourne (vx, vy, vz) ou None si pas de cible."""
    raise NotImplementedError
'''

FILES["behaviors/pick_place.py"] = r'''"""Approche, saisie, transport, depose (Bilal)."""


def approche(ep):
    raise NotImplementedError


def saisir(ep):
    raise NotImplementedError


def deposer(ep):
    raise NotImplementedError
'''

# --------------------------------------------------------------------------- mission
FILES["mission/__init__.py"] = ""

FILES["mission/arbiter.py"] = r'''"""Arbitre : securite > evitement > suivi/navigation > manipulation."""

ARRET = (0.0, 0.0, 0.0)


def choisir(commandes):
    """`commandes` : liste (par ordre de PRIORITE decroissante) de (vx, vy, vz) ou None.
    Retourne la premiere commande non nulle, sinon l'arret."""
    for cmd in commandes:
        if cmd is not None:
            return cmd
    return ARRET
'''

FILES["mission/state_machine.py"] = r'''"""Machine a etats : INIT -> CHERCHER -> APPROCHER -> SAISIR -> NAVIGUER -> DEPOSER -> FIN (+ ERREUR)."""


class Mission:
    def run(self):
        raise NotImplementedError
'''

# --------------------------------------------------------------------------- scenarios
FILES["scenarios/__init__.py"] = ""

FILES["scenarios/s1_navigation.py"] = r'''"""Scenario 1 : avance + evitement d'obstacle. Tourne deja en simulation :
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
'''

FILES["scenarios/s2_tracking.py"] = r'''"""Scenario 2 : suivi de personne (Mohamed)."""


def main():
    print("S2 : a implementer (detector -> tracker -> follow_person)")
'''

FILES["scenarios/s3_manipulation.py"] = r'''"""Scenario 3 : prise et depose d'objet (Bilal)."""


def main():
    print("S3 : a implementer (object_locator -> arm_ctrl -> pick_place)")
'''

FILES["scenarios/s4_mission_complete.py"] = r'''"""Mission complete : prendre en A, aller en B en evitant, deposer (Pierre-Olivier)."""


def main():
    print("Mission : a implementer (mission/state_machine.py)")
'''

# --------------------------------------------------------------------------- tests
FILES["tests/__init__.py"] = ""
FILES["tests/unit/__init__.py"] = ""
FILES["tests/robot/__init__.py"] = ""

FILES["tests/unit/test_arbiter.py"] = r'''from mission.arbiter import ARRET, choisir


def test_premiere_commande_non_nulle():
    assert choisir([None, (0, 0, 40), (0.3, 0, 0)]) == (0, 0, 40)


def test_arret_si_rien():
    assert choisir([None, None]) == ARRET
'''

FILES["tests/unit/test_avoid_obstacles.py"] = r'''import config
from behaviors import avoid_obstacles


def test_obstacle_proche_tourne():
    cmd = avoid_obstacles.step(config.SEUIL_OBSTACLE_CM - 1)
    assert cmd[0] == 0.0 and cmd[2] != 0.0


def test_voie_libre_ne_fait_rien():
    assert avoid_obstacles.step(config.SEUIL_OBSTACLE_CM + 50) is None
'''

FILES["tests/unit/test_pid.py"] = r'''from control.pid import PID


def test_proportionnel():
    assert PID(kp=2.0).step(3.0) == 6.0


def test_saturation_et_zone_morte():
    pid = PID(kp=10.0, zone_morte=1.0, sortie_max=5.0)
    assert pid.step(0.5) == 0.0
    assert pid.step(10.0) == 5.0
'''

FILES["tests/unit/test_sim_robot.py"] = r'''import time

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
'''

FILES["tests/robot/test_connexion.py"] = r'''"""Test AVEC robot : pytest -m robot  (PC connecte au Wi-Fi RMEP-xxxxxx)."""
import os

import pytest

pytestmark = pytest.mark.robot


def test_connexion_et_version():
    os.environ["ROBOT_MODE"] = "real"
    import importlib
    import config
    importlib.reload(config)
    from core.robot_io import robot_session
    with robot_session() as ep:
        version = ep.get_version()
        print("Version robot :", version)
        assert version
'''

# --------------------------------------------------------------------------- tools / docs / data
FILES["tools/calibration_bras.py"] = r'''"""Aide a la calibration des positions x/y du bras (Bilal). A ecrire."""

if __name__ == "__main__":
    print("A implementer : deplacer le bras et afficher les positions a reporter dans config.POS_BRAS")
'''

FILES["tools/rejouer_video.py"] = r'''"""Rejoue une video enregistree au labo : python tools/rejouer_video.py data/essai.mp4"""
import sys

import cv2


def main(chemin):
    cap = cv2.VideoCapture(chemin)
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        cv2.imshow("rejeu", frame)
        if cv2.waitKey(30) & 0xFF == ord("q"):
            break
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main(sys.argv[1])
'''

FILES["docs/README.md"] = r'''# Documentation
Schemas d'architecture, decisions, aide pour la soutenance.
'''

for _d in ("data", "models", "logs"):
    FILES[_d + "/.gitkeep"] = ""

# --------------------------------------------------------------------------- .github
FILES[".github/CODEOWNERS"] = r'''# REMPLACER les pseudos par les vrais pseudos GitHub de chacun.
/core/safety.py                      @pseudo_Lenaic
/core/sensors.py                     @pseudo_Lenaic
/control/chassis_ctrl.py             @pseudo_Lenaic
/behaviors/avoid_obstacles.py        @pseudo_Lenaic
/behaviors/navigate_a_to_b.py        @pseudo_Lenaic
/scenarios/s1_navigation.py          @pseudo_Lenaic

/core/camera.py                      @pseudo_Mohamed
/core/logger.py                      @pseudo_Mohamed
/perception/detector.py              @pseudo_Mohamed
/perception/tracker.py               @pseudo_Mohamed
/control/pid.py                      @pseudo_Mohamed
/behaviors/follow_person.py          @pseudo_Mohamed
/scenarios/s2_tracking.py            @pseudo_Mohamed

/perception/object_locator.py        @pseudo_Bilal
/control/arm_ctrl.py                 @pseudo_Bilal
/behaviors/pick_place.py             @pseudo_Bilal
/scenarios/s3_manipulation.py        @pseudo_Bilal
/tools/calibration_bras.py           @pseudo_Bilal

/main.py                             @PierreOlivier-28
/config.py                           @PierreOlivier-28
/core/robot_io.py                    @PierreOlivier-28
/sim/                                @PierreOlivier-28
/mission/                            @PierreOlivier-28
/scenarios/s4_mission_complete.py    @PierreOlivier-28
/.github/                            @PierreOlivier-28
'''

FILES[".github/pull_request_template.md"] = r'''## Ce que fait cette PR


## Checklist
- [ ] `pytest` passe (mode simulation)
- [ ] Teste sur le robot : oui / non / a faire (si non : label `a-tester-robot`)
- [ ] Interfaces (signatures) non cassees
- [ ] Registre IA mis a jour si de l'IA a ete utilisee
'''

GITIGNORE_LIGNES = [
    "venv/", ".venv/", "__pycache__/", "*.pyc", ".pytest_cache/",
    "models/*.pt", "logs/*", "!logs/.gitkeep",
]


def ecrire(chemin, contenu):
    if os.path.exists(chemin):
        print("  existe   ", chemin)
        return
    dossier = os.path.dirname(chemin)
    if dossier:
        os.makedirs(dossier, exist_ok=True)
    with open(chemin, "w", encoding="utf-8", newline="\n") as f:
        f.write(contenu)
    print("  cree     ", chemin)


def main():
    print("Creation du squelette dans", os.getcwd())
    for chemin in sorted(FILES):
        ecrire(chemin, FILES[chemin])

    existant = ""
    if os.path.exists(".gitignore"):
        with open(".gitignore", encoding="utf-8") as f:
            existant = f.read()
    manquantes = [l for l in GITIGNORE_LIGNES if l not in existant.splitlines()]
    if manquantes:
        with open(".gitignore", "a", encoding="utf-8", newline="\n") as f:
            if existant and not existant.endswith("\n"):
                f.write("\n")
            f.write("\n# --- projet RoboMaster\n" + "\n".join(manquantes) + "\n")
        print("  complete  .gitignore (+{} lignes)".format(len(manquantes)))

    print("\nTermine. Etapes suivantes :  pytest   puis   python main.py s1")


if __name__ == "__main__":
    sys.exit(main())