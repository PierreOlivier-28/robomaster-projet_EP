"""Journal des essais et sauvegarde d'images/videos pour rejouer chez soi."""
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
