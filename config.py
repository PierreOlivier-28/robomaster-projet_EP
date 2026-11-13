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
