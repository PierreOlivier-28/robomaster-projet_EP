"""Tous les reglages du projet. Aucun nombre magique dans le reste du code."""
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

# --- Simulateur 2D (sim/world2d.py) et navigation
PORT_DROITE = 1             # a verifier sur le robot reel
PORT_ARRIERE = 2
PORT_GAUCHE = 3
TOF_ANGLES_DEG = [0.0, 90.0, 180.0, -90.0]   # angle de chaque port par rapport a l'avant (horaire +)
ROBOT_RAYON_M = 0.17
SIM_ARENE_M = (4.0, 3.0)
CAMERA_FOV_DEG = 80.0
CAMERA_LARGEUR_PX = 640
CAMERA_HAUTEUR_PX = 360
LARGEUR_CIBLE_PX = 190      # largeur de la personne dans l'image a la bonne distance (~1 m)
TOLERANCE_ARRIVEE_M = 0.15
VITESSE_LATERALE = 0.25
KP_ROT_NAV = 1.0            # deg/s par degre d'erreur de cap

# --- Manipulation (S3) et mission : valeurs de depart, a CALIBRER sur le vrai robot
POS_BRAS = {                # position (x, y) du bras en mm (remplace les (0, 0) du debut du fichier)
    "repos": (80, 60),
    "prise": (180, -30),
    "depose": (180, 20),
}
TOL_BRAS_MM = 15            # tolerance pour dire que le bras est "en position de prise"
OBJET_RAYON_M = 0.04        # objet a saisir : petit cube / cylindre d'environ 8 cm
DIST_PRISE_CM = 15          # distance ToF avant visee pour saisir l'objet
TOL_DIST_PRISE_CM = 4
PORTEE_PRISE_M = 0.20       # portee de la pince devant le robot
PUISSANCE_PINCE = 50
VITESSE_RECHERCHE_DEG = 25  # rotation de recherche de l'objet (deg/s, horaire)
