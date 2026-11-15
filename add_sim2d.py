#!/usr/bin/env python
"""Ajoute le simulateur 2D au projet (vue de dessus, type turtlesim) :
evitement A->B (decalage lateral), tracking, prise d'objet et mission complete.

Usage : a la RACINE du depot (la ou se trouve config.py), venv actif :
    python add_sim2d.py
    python -m pytest
    python main.py sim

- Les fichiers du simulateur et les tests sont crees (ou mis a jour s'ils existent deja).
- Fichiers existants REMPLACES par une premiere version fonctionnelle :
  main.py, core/sensors.py, tests/unit/test_avoid_obstacles.py, behaviors/avoid_obstacles.py, behaviors/navigate_a_to_b.py,
  behaviors/follow_person.py, behaviors/pick_place.py, control/arm_ctrl.py,
  mission/state_machine.py  (verifie avec `git diff` avant de commiter).
- config.py : deux blocs de constantes sont ajoutes a la fin (s'ils n'y sont pas deja).
  Le 2e bloc redefinit POS_BRAS (valeurs de depart a calibrer sur le vrai robot).
- Relancer le script est sans danger.
"""
import os
import sys

NOUVEAUX = {}
REMPLACES = {}
NOUVEAUX["sim/world2d.py"] = r'''"""Monde 2D simule, vue de dessus. Aucune dependance graphique : testable sans fenetre.

Conventions (identiques au SDK du robot, pour que le code se transfere tel quel) :
- commande drive_speed(x, y, z) : x = avant (m/s), y = DROITE (m/s), z = rotation
  HORAIRE (deg/s)
- ecran : x vers la droite, y vers le BAS (donc un angle qui augmente tourne dans le
  sens horaire a l'ecran)
- odometrie : position relative au point de depart, x avant / y droite / yaw horaire
"""
import math
import random

import config


def norm_deg(a):
    """Ramene un angle dans ]-180, 180]."""
    while a > 180.0:
        a -= 360.0
    while a <= -180.0:
        a += 360.0
    return a


def _rayon_rect(ox, oy, dx, dy, rect):
    """Distance (en m, depuis l'origine) du rayon au rectangle, ou None."""
    x0, y0, x1, y1 = rect
    tmin, tmax = -1e9, 1e9
    for o, d, a, b in ((ox, dx, x0, x1), (oy, dy, y0, y1)):
        if abs(d) < 1e-9:
            if o < a or o > b:
                return None
        else:
            t1, t2 = (a - o) / d, (b - o) / d
            if t1 > t2:
                t1, t2 = t2, t1
            tmin, tmax = max(tmin, t1), min(tmax, t2)
            if tmin > tmax:
                return None
    if tmax < 0:
        return None
    return tmin if tmin >= 0 else 0.0


def _rayon_cercle(ox, oy, dx, dy, cx, cy, r):
    fx, fy = ox - cx, oy - cy
    b = fx * dx + fy * dy
    c = fx * fx + fy * fy - r * r
    disc = b * b - c
    if disc < 0:
        return None
    t = -b - math.sqrt(disc)
    if t < 0:
        t = -b + math.sqrt(disc)
    return t if t >= 0 else None


class World:
    def __init__(self, largeur=None, hauteur=None, seed=0):
        larg, haut = config.SIM_ARENE_M
        self.W = largeur or larg
        self.H = hauteur or haut
        self.rayon = config.ROBOT_RAYON_M
        self.obstacles = []              # rectangles (x0, y0, x1, y1) en metres
        self.person = None               # [x, y]
        self.person_start = None
        self.rayon_personne = 0.2
        self.person_path = []
        self.person_marche = False
        self.person_vitesse = 0.2
        self._person_idx = 0
        self.bruit_cm = 0.0
        self.portee_m = 3.0
        self.rng = random.Random(seed)
        # --- manipulation (S3) : un objet a saisir, une zone de depose, un bras et une pince
        self.objet = None                # [x, y] au sol (ou devant le robot s'il est tenu)
        self.objet_depart = None
        self.objet_rayon = config.OBJET_RAYON_M
        self.objet_actif = False         # False : l'objet n'existe pas (modes 1-3 du simulateur)
        self.objet_tenu = False
        self.zone_depose = None          # rectangle (x0, y0, x1, y1)
        self.bras = tuple(config.POS_BRAS["repos"])     # position du bras (mm)
        self.pince_ouverte = True
        self.reset()

    # ------------------------------------------------------------ construction
    @classmethod
    def demo(cls):
        """Terrain de demonstration : 2 obstacles entre A et B, une personne qui marche."""
        w = cls()
        w.obstacles = [(1.5, 1.1, 1.8, 1.9), (2.5, 0.7, 2.8, 1.5)]
        w.set_person(3.3, 2.6,
                     chemin=[(3.3, 2.6), (3.3, 0.3), (2.1, 0.3), (2.1, 2.6)])
        w.set_objet(1.1, 2.25)
        w.zone_depose = (3.0, 1.0, 4.0, 2.0)          # autour du but B
        return w

    # ------------------------------------------------------------ objet / bras / pince
    def set_objet(self, x, y):
        self.objet = [x, y]
        self.objet_depart = [x, y]
        self.objet_tenu = False

    def activer_objet(self, actif=True):
        self.objet_actif = actif and self.objet is not None

    def _objet_au_sol(self):
        return self.objet if (self.objet_actif and not self.objet_tenu) else None

    def _point_devant(self, distance):
        x, y, th = self.pose
        return [x + distance * math.cos(th), y + distance * math.sin(th)]

    def distance_objet_cm(self):
        """Distance entre le bord du robot et la surface de l'objet (cm), ou None."""
        if not self.objet_actif or self.objet is None or self.objet_tenu:
            return None
        d = math.hypot(self.objet[0] - self.pose[0], self.objet[1] - self.pose[1])
        return max(0.0, (d - self.rayon - self.objet_rayon) * 100.0)

    def _angle_objet_deg(self):
        x, y, th = self.pose
        return norm_deg(math.degrees(math.atan2(self.objet[1] - y, self.objet[0] - x) - th))

    def bras_aller(self, x_mm, y_mm):
        self.bras = (x_mm, y_mm)

    def bras_en_prise(self):
        px, py = config.POS_BRAS["prise"]
        return (abs(self.bras[0] - px) <= config.TOL_BRAS_MM and
                abs(self.bras[1] - py) <= config.TOL_BRAS_MM)

    def pince_fermer(self):
        """Ferme la pince. L'objet est saisi si le bras est en position de prise et si
        l'objet est devant la pince, a portee."""
        d = self.distance_objet_cm()
        if (self.pince_ouverte and d is not None and self.bras_en_prise()
                and d <= config.PORTEE_PRISE_M * 100.0
                and abs(self._angle_objet_deg()) <= 12.0):
            self.objet_tenu = True
        self.pince_ouverte = False

    def pince_ouvrir(self):
        """Ouvre la pince ; si le robot tenait l'objet, il le depose devant lui."""
        if self.objet_tenu:
            self.objet_tenu = False
            self.objet = self._point_devant(self.rayon + 0.10)
        self.pince_ouverte = True

    def objet_dans_zone(self):
        if self.objet is None or self.zone_depose is None or self.objet_tenu:
            return False
        x0, y0, x1, y1 = self.zone_depose
        return x0 <= self.objet[0] <= x1 and y0 <= self.objet[1] <= y1

    def set_person(self, x, y, chemin=None):
        self.person = [x, y]
        self.person_start = [x, y]
        self.person_path = list(chemin or [])
        self._person_idx = 0

    def placer_personne_devant(self, distance=1.6):
        """Place la personne devant le robot (visible, sans obstacle) et la fait marcher
        autour de l'arene. Retourne False si aucune place libre n'a ete trouvee."""
        x, y, th = self.pose
        for d in (distance, 1.3, 1.0, 2.0, 0.8):
            for dang in (0.0, 15.0, -15.0, 30.0, -30.0):
                a = th + math.radians(dang)
                px, py = x + d * math.cos(a), y + d * math.sin(a)
                m = self.rayon_personne
                if not (m <= px <= self.W - m and m <= py <= self.H - m):
                    continue
                if any(r[0] - m < px < r[2] + m and r[1] - m < py < r[3] + m
                       for r in self.obstacles):
                    continue
                self.person = [px, py]
                if self.boite_personne() is None:
                    continue
                marge = 0.45
                coins = [(self.W - marge, marge), (self.W - marge, self.H - marge),
                         (marge, self.H - marge), (marge, marge)]
                k = min(range(4), key=lambda i: math.hypot(coins[i][0] - px, coins[i][1] - py))
                self.person_path = coins[k:] + coins[:k]
                self._person_idx = 0
                self.person_start = [px, py]
                return True
        return False

    def ajouter_obstacle(self, x, y, taille=0.4):
        h = taille / 2.0
        self.obstacles.append((x - h, y - h, x + h, y + h))

    def retirer_obstacle_en(self, x, y):
        for r in list(self.obstacles):
            if r[0] <= x <= r[2] and r[1] <= y <= r[3]:
                self.obstacles.remove(r)
                return True
        return False

    def reset(self):
        self.pose = [0.5, self.H / 2.0, 0.0]          # x, y (m), cap (rad, horaire)
        self.start = tuple(self.pose)
        self.t = 0.0
        self.cmd = (0.0, 0.0, 0.0)
        self.cmd_expire = 0.0
        self.trace = []
        self.collision = False
        self._person_idx = 0
        if self.person_start is not None:
            self.person = list(self.person_start)
        if self.objet_depart is not None:
            self.objet = list(self.objet_depart)
        self.objet_tenu = False
        self.bras = tuple(config.POS_BRAS["repos"])
        self.pince_ouverte = True

    # ------------------------------------------------------------ commandes
    def set_cmd(self, vx, vy, vz, timeout=1.0):
        """Meme comportement que le robot : la commande expire apres `timeout` secondes."""
        self.cmd = (vx, vy, vz)
        self.cmd_expire = self.t + timeout

    # ------------------------------------------------------------ physique
    def _libre(self, x, y):
        r = self.rayon
        if x < r or y < r or x > self.W - r or y > self.H - r:
            return False
        for (x0, y0, x1, y1) in self.obstacles:
            px = min(max(x, x0), x1)
            py = min(max(y, y0), y1)
            if (x - px) ** 2 + (y - py) ** 2 < r * r:
                return False
        if self.person is not None:
            if math.hypot(x - self.person[0], y - self.person[1]) < r + self.rayon_personne:
                return False
        o = self._objet_au_sol()
        if o is not None and math.hypot(x - o[0], y - o[1]) < r + self.objet_rayon:
            return False
        return True

    def step(self, dt):
        self.t += dt
        vx, vy, vz = self.cmd if self.t <= self.cmd_expire else (0.0, 0.0, 0.0)
        x, y, th = self.pose
        fx, fy = math.cos(th), math.sin(th)       # avant
        rx, ry = -math.sin(th), math.cos(th)      # droite
        nx = x + (vx * fx + vy * rx) * dt
        ny = y + (vx * fy + vy * ry) * dt
        self.collision = False
        if self._libre(nx, ny):
            x, y = nx, ny
        else:
            self.collision = True
            if self._libre(nx, y):
                x = nx
            elif self._libre(x, ny):
                y = ny
        th += math.radians(vz) * dt
        self.pose = [x, y, th]
        if not self.trace or math.hypot(x - self.trace[-1][0], y - self.trace[-1][1]) > 0.02:
            self.trace.append((x, y))
            if len(self.trace) > 3000:
                self.trace.pop(0)
        self._avancer_personne(dt)
        if self.objet_tenu and self.objet is not None:
            self.objet = self._point_devant(self.rayon + 0.10)

    def _avancer_personne(self, dt):
        if not (self.person_marche and self.person is not None and self.person_path):
            return
        cx, cy = self.person
        tx, ty = self.person_path[self._person_idx]
        d = math.hypot(tx - cx, ty - cy)
        pas = self.person_vitesse * dt
        if d <= pas:
            self.person = [tx, ty]
            self._person_idx = (self._person_idx + 1) % len(self.person_path)
        else:
            self.person = [cx + (tx - cx) / d * pas, cy + (ty - cy) / d * pas]

    # ------------------------------------------------------------ capteurs
    def _distance_rayon(self, angle_rad, max_m, avec_personne=True, avec_objet=True):
        """Distance depuis le centre du robot jusqu'au premier objet, ou None."""
        ox, oy = self.pose[0], self.pose[1]
        dx, dy = math.cos(angle_rad), math.sin(angle_rad)
        meilleur = None

        def maj(t):
            nonlocal meilleur
            if t is not None and (meilleur is None or t < meilleur):
                meilleur = t

        # murs de l'arene
        t_mur = 1e9
        if dx > 1e-9:
            t_mur = min(t_mur, (self.W - ox) / dx)
        elif dx < -1e-9:
            t_mur = min(t_mur, (0.0 - ox) / dx)
        if dy > 1e-9:
            t_mur = min(t_mur, (self.H - oy) / dy)
        elif dy < -1e-9:
            t_mur = min(t_mur, (0.0 - oy) / dy)
        maj(t_mur)
        for rect in self.obstacles:
            maj(_rayon_rect(ox, oy, dx, dy, rect))
        if avec_personne and self.person is not None:
            maj(_rayon_cercle(ox, oy, dx, dy, self.person[0], self.person[1],
                              self.rayon_personne))
        o = self._objet_au_sol()
        if avec_objet and o is not None:
            maj(_rayon_cercle(ox, oy, dx, dy, o[0], o[1], self.objet_rayon))
        if meilleur is None or meilleur > max_m:
            return None
        return meilleur

    def tof_cm(self):
        """Distances des 4 capteurs ToF (cm, depuis le bord du robot ; 999 = rien)."""
        res = []
        for ang in config.TOF_ANGLES_DEG:
            d = self._distance_rayon(self.pose[2] + math.radians(ang),
                                     self.portee_m + self.rayon)
            if d is None:
                res.append(config.DISTANCE_INVALIDE_CM)
            else:
                cm = max(0.0, (d - self.rayon) * 100.0)
                if self.bruit_cm:
                    cm = max(0.0, cm + self.rng.gauss(0.0, self.bruit_cm))
                res.append(cm)
        return res

    def tof_mm(self):
        """Meme format que le SDK : 4 valeurs en mm (3000 = invalide / rien)."""
        return [3000.0 if v >= config.DISTANCE_INVALIDE_CM else v * 10.0
                for v in self.tof_cm()]

    def odometrie(self):
        """(x avant, y droite, yaw horaire en deg) relatifs au point de depart."""
        x0, y0, th0 = self.start
        dx, dy = self.pose[0] - x0, self.pose[1] - y0
        c, s = math.cos(th0), math.sin(th0)
        return (dx * c + dy * s, -dx * s + dy * c,
                norm_deg(math.degrees(self.pose[2] - th0)))

    def odom_vers_monde(self, x, y):
        x0, y0, th0 = self.start
        c, s = math.cos(th0), math.sin(th0)
        return (x0 + x * c - y * s, y0 + x * s + y * c)

    def monde_vers_odom(self, wx, wy):
        x0, y0, th0 = self.start
        dx, dy = wx - x0, wy - y0
        c, s = math.cos(th0), math.sin(th0)
        return (dx * c + dy * s, -dx * s + dy * c)

    def boite_personne(self):
        """Boite (x1, y1, x2, y2) de la personne dans l'image de la camera, ou None
        (hors champ ou cachee par un obstacle)."""
        if self.person is None:
            return None
        x, y, th = self.pose
        dx, dy = self.person[0] - x, self.person[1] - y
        d = math.hypot(dx, dy)
        if d < 1e-6:
            return None
        ang = norm_deg(math.degrees(math.atan2(dy, dx) - th))
        demi = config.CAMERA_FOV_DEG / 2.0
        if abs(ang) >= demi - 1.0:
            return None
        dh = self._distance_rayon(math.atan2(dy, dx), d, avec_personne=False)
        if dh is not None and dh < d - self.rayon_personne:
            return None
        wpx, hpx = config.CAMERA_LARGEUR_PX, config.CAMERA_HAUTEUR_PX
        f = (wpx / 2.0) / math.tan(math.radians(demi))
        profondeur = d * math.cos(math.radians(ang))
        cx = wpx / 2.0 + f * math.tan(math.radians(ang))
        larg = f * 0.5 / max(profondeur, 0.05)          # epaules ~ 0,5 m
        haut = min(float(hpx), larg * 2.4)
        return (cx - larg / 2.0, hpx / 2.0 - haut / 2.0,
                cx + larg / 2.0, hpx / 2.0 + haut / 2.0)

    def boite_objet(self):
        """Boite (x1, y1, x2, y2) de l'objet dans l'image (cube de ~6 cm pose au sol), ou None."""
        o = self._objet_au_sol()
        if o is None:
            return None
        x, y, th = self.pose
        dx, dy = o[0] - x, o[1] - y
        d = math.hypot(dx, dy)
        if d < 1e-6:
            return None
        ang = norm_deg(math.degrees(math.atan2(dy, dx) - th))
        demi = config.CAMERA_FOV_DEG / 2.0
        if abs(ang) >= demi - 1.0:
            return None
        dh = self._distance_rayon(math.atan2(dy, dx), d, avec_personne=False, avec_objet=False)
        if dh is not None and dh < d - self.objet_rayon:
            return None
        wpx, hpx = config.CAMERA_LARGEUR_PX, config.CAMERA_HAUTEUR_PX
        f = (wpx / 2.0) / math.tan(math.radians(demi))
        profondeur = max(d * math.cos(math.radians(ang)), 0.05)
        cx = wpx / 2.0 + f * math.tan(math.radians(ang))
        cote = f * 0.06 / profondeur
        cy = hpx * 0.72
        return (cx - cote / 2.0, cy - cote / 2.0, cx + cote / 2.0, cy + cote / 2.0)
'''
NOUVEAUX["sim/viewer.py"] = r'''"""Fenetre du simulateur 2D (OpenCV) : dessin + souris. Aucune logique de robot ici."""
import math

import cv2
import numpy as np

import config

ECHELLE = 200          # pixels par metre
PANNEAU = 300          # largeur du panneau d'information (px)

FOND = (38, 38, 38)
GRILLE = (52, 52, 52)
MUR = (210, 210, 210)
OBSTACLE = (130, 130, 140)
PERSONNE = (0, 165, 255)
BUT = (80, 200, 80)
ROBOT = (235, 160, 60)
TRACE = (150, 105, 50)
RAYON_OK = (90, 200, 90)
RAYON_PROCHE = (60, 60, 235)
TEXTE = (230, 230, 230)
TEXTE_DOUX = (160, 160, 160)
ROUGE = (60, 60, 240)
OBJET = (60, 220, 240)
ZONE = (90, 160, 90)


def _px(x, y):
    return int(round(x * ECHELLE)), int(round(y * ECHELLE))


class Viewer:
    def __init__(self, world, titre="Simulateur RoboMaster 2D"):
        self.world = world
        self.titre = titre
        self.largeur = int(world.W * ECHELLE) + PANNEAU
        self.hauteur = max(int(world.H * ECHELLE), 560)
        self.evenements = []        # ("gauche"|"glisse"|"relache"|"droit", x_m, y_m)
        self._bouton = False
        cv2.namedWindow(self.titre, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(self.titre, self._souris)

    # ------------------------------------------------------------ souris
    def _souris(self, event, px, py, flags, param):
        if px >= self.world.W * ECHELLE or py >= self.world.H * ECHELLE:
            return
        x, y = px / ECHELLE, py / ECHELLE
        if event == cv2.EVENT_LBUTTONDOWN:
            self._bouton = True
            self.evenements.append(("gauche", x, y))
        elif event == cv2.EVENT_MOUSEMOVE and self._bouton:
            self.evenements.append(("glisse", x, y))
        elif event == cv2.EVENT_LBUTTONUP:
            self._bouton = False
            self.evenements.append(("relache", x, y))
        elif event == cv2.EVENT_RBUTTONDOWN:
            self.evenements.append(("droit", x, y))

    def prendre_evenements(self):
        ev, self.evenements = self.evenements, []
        return ev

    # ------------------------------------------------------------ dessin
    def render(self, info):
        """info : dict avec mode, cmd, tof (cm), pose_odom, but (x,y monde), etat, aide."""
        w = self.world
        img = np.full((self.hauteur, self.largeur, 3), FOND, np.uint8)
        aw, ah = int(w.W * ECHELLE), int(w.H * ECHELLE)

        # quadrillage tous les 0,5 m
        for k in range(1, int(w.W / 0.5) + 1):
            cv2.line(img, (int(k * 0.5 * ECHELLE), 0), (int(k * 0.5 * ECHELLE), ah), GRILLE, 1)
        for k in range(1, int(w.H / 0.5) + 1):
            cv2.line(img, (0, int(k * 0.5 * ECHELLE)), (aw, int(k * 0.5 * ECHELLE)), GRILLE, 1)
        cv2.rectangle(img, (0, 0), (aw - 1, ah - 1), MUR, 2)

        if w.objet_actif and w.zone_depose is not None:
            zx0, zy0, zx1, zy1 = w.zone_depose
            ov = img.copy()
            cv2.rectangle(ov, _px(zx0, zy0), _px(zx1, zy1), ZONE, -1)
            cv2.addWeighted(ov, 0.25, img, 0.75, 0, img)
            cv2.rectangle(img, _px(zx0, zy0), _px(zx1, zy1), ZONE, 1)
            cv2.putText(img, "zone de depose", (_px(zx0, zy0)[0] + 8, _px(zx0, zy0)[1] + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, ZONE, 1)
        for (x0, y0, x1, y1) in w.obstacles:
            cv2.rectangle(img, _px(x0, y0), _px(x1, y1), OBSTACLE, -1)
            cv2.rectangle(img, _px(x0, y0), _px(x1, y1), (180, 180, 190), 1)

        # but
        if info.get("but") is not None:
            bx, by = _px(*info["but"])
            cv2.drawMarker(img, (bx, by), BUT, cv2.MARKER_CROSS, 22, 2)
            cv2.circle(img, (bx, by), int(config.TOLERANCE_ARRIVEE_M * ECHELLE), BUT, 1)
            cv2.putText(img, "B", (bx + 10, by - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, BUT, 2)
        # depart
        sx, sy = _px(w.start[0], w.start[1])
        cv2.putText(img, "A", (sx - 6, sy + 28 + int(w.rayon * ECHELLE)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, TEXTE_DOUX, 1)

        # trace
        if len(w.trace) > 1:
            pts = np.array([_px(x, y) for x, y in w.trace], np.int32)
            cv2.polylines(img, [pts], False, TRACE, 1, cv2.LINE_AA)

        x, y, th = w.pose
        # champ de vision de la camera
        demi = math.radians(config.CAMERA_FOV_DEG / 2.0)
        for sgn in (-1, 1):
            a = th + sgn * demi
            cv2.line(img, _px(x, y), _px(x + 1.2 * math.cos(a), y + 1.2 * math.sin(a)),
                     (85, 85, 85), 1, cv2.LINE_AA)

        # rayons ToF
        tof = info.get("tof") or w.tof_cm()
        for ang, cm in zip(config.TOF_ANGLES_DEG, tof):
            a = th + math.radians(ang)
            invalide = cm >= config.DISTANCE_INVALIDE_CM
            d = w.portee_m if invalide else cm / 100.0
            ex = x + (w.rayon + d) * math.cos(a)
            ey = y + (w.rayon + d) * math.sin(a)
            couleur = (95, 95, 95) if invalide else (
                RAYON_PROCHE if cm < config.SEUIL_OBSTACLE_CM else RAYON_OK)
            cv2.line(img, _px(x + w.rayon * math.cos(a), y + w.rayon * math.sin(a)),
                     _px(ex, ey), couleur, 1, cv2.LINE_AA)
            if not invalide:
                cv2.circle(img, _px(ex, ey), 4, couleur, -1)

        # personne
        if w.person is not None:
            cv2.circle(img, _px(*w.person), int(w.rayon_personne * ECHELLE), PERSONNE, -1)
            ppx, ppy = _px(*w.person)
            cv2.putText(img, "personne",
                        (ppx - 32, ppy - int(w.rayon_personne * ECHELLE) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, PERSONNE, 1)

        # objet a saisir (au sol, ou tenu devant le robot)
        if w.objet_actif and w.objet is not None:
            ox, oy = _px(*w.objet)
            cv2.rectangle(img, (ox - 9, oy - 9), (ox + 9, oy + 9), OBJET, -1)
            cv2.rectangle(img, (ox - 9, oy - 9), (ox + 9, oy + 9), (255, 255, 255), 1)

        # robot
        couleur = ROUGE if w.collision else ROBOT
        cv2.circle(img, _px(x, y), int(w.rayon * ECHELLE), couleur, -1)
        cv2.circle(img, _px(x, y), int(w.rayon * ECHELLE), (255, 255, 255), 1)
        cv2.arrowedLine(img, _px(x, y),
                        _px(x + 0.9 * w.rayon * math.cos(th), y + 0.9 * w.rayon * math.sin(th)),
                        (255, 255, 255), 2, tipLength=0.4)

        self._panneau(img, info, aw)
        return img

    def _panneau(self, img, info, x0):
        w = self.world
        cv2.rectangle(img, (x0, 0), (self.largeur, self.hauteur), (28, 28, 28), -1)
        y = 24

        def ligne(txt, couleur=TEXTE, taille=0.5, pas=22):
            nonlocal y
            cv2.putText(img, txt, (x0 + 14, y), cv2.FONT_HERSHEY_SIMPLEX, taille, couleur, 1,
                        cv2.LINE_AA)
            y += pas

        ligne(self.titre, (255, 255, 255), 0.55, 28)
        ligne("MODE : " + info.get("mode", "?"), (120, 210, 255), 0.55, 26)
        if info.get("etat"):
            ligne("etat : " + info["etat"], TEXTE_DOUX)
        cmd = info.get("cmd") or (0.0, 0.0, 0.0)
        ligne("cmd  vx={:+.2f} vy={:+.2f} vz={:+.0f}".format(*cmd))
        tof = info.get("tof") or w.tof_cm()
        noms = ["avant", "droite", "arriere", "gauche"]
        for nom, cm in zip(noms, tof):
            txt = "ToF {:8s}: {}".format(nom, "---" if cm >= config.DISTANCE_INVALIDE_CM
                                         else "{:3.0f} cm".format(cm))
            ligne(txt, RAYON_PROCHE if cm < config.SEUIL_OBSTACLE_CM else TEXTE)
        po = info.get("pose_odom")
        if po:
            ligne("odo  x={:+.2f} y={:+.2f} yaw={:+.0f}".format(*po), TEXTE_DOUX)
        if w.objet_actif:
            ligne("bras x={:.0f} y={:.0f} mm  pince {}".format(
                w.bras[0], w.bras[1], "ouverte" if w.pince_ouverte else "fermee"), TEXTE_DOUX)
            if w.objet_tenu:
                etat_obj = "tenu par la pince"
            elif w.objet_dans_zone():
                etat_obj = "DEPOSE DANS LA ZONE"
            else:
                etat_obj = "au sol"
            ligne("objet : " + etat_obj, OBJET)
        ligne("t = {:.1f} s".format(w.t), TEXTE_DOUX)
        if w.collision:
            ligne("!! CONTACT / COLLISION !!", ROUGE, 0.55, 26)
        else:
            y += 4

        # mini-vue camera
        y += 4
        ligne("camera (detection simulee)", TEXTE_DOUX, 0.45, 18)
        cw = PANNEAU - 28
        ch = int(cw * config.CAMERA_HAUTEUR_PX / config.CAMERA_LARGEUR_PX)
        cx0, cy0 = x0 + 14, y
        cv2.rectangle(img, (cx0, cy0), (cx0 + cw, cy0 + ch), (12, 12, 12), -1)
        cv2.rectangle(img, (cx0, cy0), (cx0 + cw, cy0 + ch), (90, 90, 90), 1)
        cv2.line(img, (cx0 + cw // 2, cy0), (cx0 + cw // 2, cy0 + ch), (60, 60, 60), 1)
        b = w.boite_personne()
        if b is not None:
            k = cw / float(config.CAMERA_LARGEUR_PX)
            cv2.rectangle(img, (int(cx0 + b[0] * k), int(cy0 + b[1] * k)),
                          (int(cx0 + b[2] * k), int(cy0 + b[3] * k)), (80, 220, 80), 2)
            cv2.putText(img, "person", (int(cx0 + b[0] * k), int(cy0 + b[1] * k) - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (80, 220, 80), 1)
        bo = w.boite_objet()
        if bo is not None:
            k = cw / float(config.CAMERA_LARGEUR_PX)
            cv2.rectangle(img, (int(cx0 + bo[0] * k), int(cy0 + bo[1] * k)),
                          (int(cx0 + bo[2] * k), int(cy0 + bo[3] * k)), OBJET, 2)
            cv2.putText(img, "objet", (int(cx0 + bo[0] * k), int(cy0 + bo[1] * k) - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, OBJET, 1)
        if b is None and bo is None:
            cv2.putText(img, "rien detecte", (cx0 + 8, cy0 + ch // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (110, 110, 110), 1)
        y = cy0 + ch + 26

        for txt in info.get("aide", []):
            ligne(txt, TEXTE_DOUX, 0.42, 17)

    def afficher(self, img, attente_ms=1):
        cv2.imshow(self.titre, img)
        return cv2.waitKey(attente_ms) & 0xFF

    def fermer(self):
        cv2.destroyAllWindows()
'''
NOUVEAUX["sim/demo.py"] = r'''"""Simulateur interactif 2D :  python main.py sim

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
'''
NOUVEAUX["sim/fake_detector.py"] = r'''"""Detecteur simule : meme format de sortie que perception/detector.py (Detector.detect).

Permet de tester le tracking et l'asservissement SANS YOLO, dans le simulateur 2D.
"""


class FakeDetector:
    def __init__(self, world):
        self.world = world

    def detect(self, frame=None):
        """Liste de boites (x1, y1, x2, y2, classe, confiance) ; vide si personne n'est visible."""
        b = self.world.boite_personne()
        if b is None:
            return []
        return [(b[0], b[1], b[2], b[3], "person", 0.9)]
'''
NOUVEAUX["sim/fake_object_locator.py"] = r'''"""Localisation d'objet simulee : meme sortie que perception/object_locator.py.

locate(frame) -> (cx, taille) en pixels (centre horizontal et largeur de l'objet), ou None.
Permet de developper la prise d'objet SANS vision, dans le simulateur 2D.
"""


class FakeObjectLocator:
    def __init__(self, world):
        self.world = world

    def locate(self, frame=None):
        b = self.world.boite_objet()
        if b is None:
            return None
        return ((b[0] + b[2]) / 2.0, b[2] - b[0])
'''
NOUVEAUX["core/odometry.py"] = r'''"""Position du robot relative a son point de depart (odometrie des roues).

pose() -> (x avant en m, y droite en m, yaw horaire en deg)
A VERIFIER au labo : signe et unite du 3e champ renvoye par ep.chassis.sub_position().
"""


class Odometrie:
    def __init__(self, ep, freq=10):
        self.ep = ep
        self.freq = freq
        self._pose = (0.0, 0.0, 0.0)

    def _callback(self, info):
        self._pose = (float(info[0]), float(info[1]), float(info[2]))

    def start(self):
        self.ep.chassis.sub_position(cs=0, freq=self.freq, callback=self._callback)

    def stop(self):
        try:
            self.ep.chassis.unsub_position()
        except Exception:
            pass

    def pose(self):
        return self._pose
'''
NOUVEAUX["tests/unit/test_world2d.py"] = r'''import math

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
'''
NOUVEAUX["tests/unit/test_navigation.py"] = r'''import math

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
'''
NOUVEAUX["tests/unit/test_manipulation.py"] = r'''import math

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
'''
NOUVEAUX["tests/unit/test_mission.py"] = r'''import config
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
'''
NOUVEAUX["tests/unit/test_avoid_obstacles.py"] = r'''import config
from behaviors import avoid_obstacles


def test_obstacle_proche_se_decale_sans_tourner():
    cmd = avoid_obstacles.step(config.SEUIL_OBSTACLE_CM - 1)
    assert cmd[0] == 0.0 and cmd[1] != 0.0 and cmd[2] == 0.0


def test_voie_libre_ne_fait_rien():
    assert avoid_obstacles.step(config.SEUIL_OBSTACLE_CM + 50) is None
'''
REMPLACES["main.py"] = r'''"""Point d'entree : python main.py s1|s2|s3|mission|sim [--real]

  s1 / s2 / s3 / mission : scenarios (faux robot en console, ou vrai robot avec --real)
  sim                    : simulateur 2D interactif (fenetre), pour voir ce que fait le code
"""
import argparse
import os


def main(argv=None):
    parser = argparse.ArgumentParser(description="Projet RoboMaster")
    parser.add_argument("scenario", choices=["s1", "s2", "s3", "mission", "sim"])
    parser.add_argument("--real", action="store_true",
                        help="utiliser le vrai robot (sinon : simulation)")
    args = parser.parse_args(argv)

    # le mode doit etre fixe AVANT d'importer config / les scenarios
    os.environ["ROBOT_MODE"] = "real" if args.real else "sim"

    if args.scenario == "sim":
        from sim import demo
        demo.main()
        return
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
REMPLACES["core/sensors.py"] = r'''"""Lecture des capteurs ToF (distances en cm, valeurs invalides filtrees)."""
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

    def cm(self, port):
        """Distance du capteur `port` en cm (config.DISTANCE_INVALIDE_CM si inconnue)."""
        if len(self._mm) <= port or not self._valide(self._mm[port]):
            return config.DISTANCE_INVALIDE_CM
        return self._mm[port] / 10.0

    def front_cm(self):
        return self.cm(config.PORT_AVANT)

    def left_cm(self):
        return self.cm(config.PORT_GAUCHE)

    def right_cm(self):
        return self.cm(config.PORT_DROITE)

    def back_cm(self):
        return self.cm(config.PORT_ARRIERE)

    def min_cm(self):
        """Plus petite distance valide parmi les 4 capteurs."""
        valides = [m / 10.0 for m in self._mm if self._valide(m)]
        return min(valides) if valides else config.DISTANCE_INVALIDE_CM
'''
REMPLACES["sim/fake_robot.py"] = r'''"""Faux robot : meme interface que le SDK robomaster (parties utilisees par le projet).

- FakeRobot()            -> console seulement (distances scriptees)
- FakeRobot(world=World) -> relie au simulateur 2D (sim/world2d.py) : les commandes
                            deplacent le robot et les capteurs lisent le monde.
"""
import threading
import time

import config
from sim import fake_tof
from sim.fake_camera import FakeCamera


class FakeChassis:
    def __init__(self, world=None):
        self.world = world
        self.commandes = []          # historique pour les tests
        self._pos_actif = False

    def drive_speed(self, x=0, y=0, z=0, timeout=None):
        self.commandes.append((x, y, z))
        if len(self.commandes) > 5000:
            del self.commandes[:2500]
        if self.world is not None:
            self.world.set_cmd(x, y, z, timeout if timeout else 1.0)

    def sub_position(self, cs=0, freq=5, callback=None):
        """Position relative au depart : (x avant m, y droite m, yaw horaire deg)."""
        self._pos_actif = True

        def boucle():
            while self._pos_actif:
                if self.world is not None:
                    callback(self.world.odometrie())
                else:
                    callback((0.0, 0.0, 0.0))
                time.sleep(1.0 / freq)

        threading.Thread(target=boucle, daemon=True).start()

    def unsub_position(self):
        self._pos_actif = False


class FakeSensor:
    def __init__(self, fonction_distance=None, world=None):
        self.world = world
        self.fonction = fonction_distance or fake_tof.onde()
        self._actif = False

    def sub_distance(self, freq=10, callback=None):
        self._actif = True
        t0 = time.time()

        def boucle():
            while self._actif:
                if self.world is not None:
                    mm = self.world.tof_mm()
                else:
                    cm = self.fonction(time.time() - t0)
                    mm = [3000.0] * 4
                    mm[config.PORT_AVANT] = cm * 10.0
                callback(mm)
                time.sleep(1.0 / freq)

        threading.Thread(target=boucle, daemon=True).start()

    def unsub_distance(self):
        self._actif = False


class _Action:
    """Meme interface que les actions du SDK : arm.moveto(...).wait_for_completed()."""

    def wait_for_completed(self, timeout=None):
        return True


class FakeArm:
    def __init__(self, world=None):
        self.world = world

    def moveto(self, x=0, y=0):
        if self.world is not None:
            self.world.bras_aller(x, y)
        else:
            print("[sim] bras -> ({}, {})".format(x, y))
        return _Action()


class FakeGripper:
    def __init__(self, world=None):
        self.world = world

    def open(self, power=50):
        if self.world is not None:
            self.world.pince_ouvrir()
        else:
            print("[sim] pince ouverte")

    def close(self, power=50):
        if self.world is not None:
            self.world.pince_fermer()
        else:
            print("[sim] pince fermee")

    def objet_saisi(self):
        """Uniquement dans la simulation : la pince tient-elle l'objet ?"""
        return True if self.world is None else self.world.objet_tenu


class FakeRobot:
    def __init__(self, fonction_distance=None, source_camera=0, world=None):
        self.world = world
        self.chassis = FakeChassis(world)
        self.sensor = FakeSensor(fonction_distance, world)
        self.camera = FakeCamera(source_camera)
        self.robotic_arm = FakeArm(world)
        self.gripper = FakeGripper(world)

    def close(self):
        self.sensor.unsub_distance()
        self.chassis.unsub_position()
        self.camera.stop_video_stream()
'''
REMPLACES["behaviors/avoid_obstacles.py"] = r'''"""Evitement reactif (Lenaic). Fonction PURE : renvoie une commande, ne pilote pas le robot.

Le robot est holonome (roues mecanum) : devant un obstacle il ne tourne pas, il se
DECALE sur le cote (translation). Convention du SDK : vy > 0 = vers la DROITE.
Pour un vrai trajet A -> B avec evitement, voir behaviors/navigate_a_to_b.py.
"""
import config


def step(avant_cm, gauche_cm=None, droite_cm=None):
    """Retourne (vx, vy, vz) si un obstacle est trop proche devant, sinon None.

    Sans capteurs lateraux : se decale toujours vers la droite.
    Avec capteurs lateraux : se decale du cote ou il y a le plus de place.
    """
    if avant_cm >= config.SEUIL_OBSTACLE_CM:
        return None
    sens = 1.0
    if gauche_cm is not None and droite_cm is not None and gauche_cm > droite_cm:
        sens = -1.0
    return (0.0, sens * config.VITESSE_LATERALE, 0.0)   # on s'arrete d'avancer et on se decale
'''
REMPLACES["behaviors/navigate_a_to_b.py"] = r'''"""Navigation A -> B avec evitement d'obstacles (Lenaic).

Le Navigateur ne pilote pas le robot : step() renvoie une commande (vx, vy, vz).
Le repere est celui de l'odometrie : x avant du depart, y droite du depart,
yaw en degres, positif vers la droite (sens horaire).

Etats : LIBRE (cap sur le but) -> STRAFE (obstacle devant : on se decale sur le cote
le plus libre) -> MARGE (on continue de se decaler pour ne pas accrocher le coin)
-> AVANCE (on longe l'obstacle) -> LIBRE.
"""
import math

import config

T_MARGE_S = 1.2        # decalage supplementaire apres que l'avant soit libre
T_DEGAGE_S = 1.2       # on avance encore un peu apres avoir depasse l'obstacle
DIST_LATERALE_CM = 60  # "l'obstacle est encore a cote de nous"
DIST_REPULSION_CM = 40  # en dessous, on s'ecarte de l'obstacle lateral


def _norm_deg(a):
    while a > 180.0:
        a -= 360.0
    while a <= -180.0:
        a += 360.0
    return a


def _clamp(v, m):
    return max(-m, min(m, v))


class Navigateur:
    def __init__(self, but, tolerance=None):
        self.but = (float(but[0]), float(but[1]))
        self.tol = tolerance or config.TOLERANCE_ARRIVEE_M
        self.arrive = False
        self.etat = "LIBRE"
        self.cote = 0              # +1 = on se decale vers la droite, -1 = vers la gauche
        self._t = 0.0              # chrono de l'etat courant
        self._t_libre = 0.0        # temps depuis lequel le cote de l'obstacle est libre
        self._vu = False           # l'obstacle a-t-il ete vu sur le cote ?
        self._yaw_ref = 0.0        # cap tenu pendant le contournement
        self._prev = None          # derniere position (detection de blocage)
        self._v_prec = 0.0         # vitesse commandee au pas precedent
        self._t_bloque = 0.0       # temps pendant lequel on avance sans bouger
        self._t_total = 0.0

    def step(self, pose, avant_cm, gauche_cm, droite_cm, dt=0.1):
        """pose = (x, y, yaw_deg). Retourne ((vx, vy, vz), arrive)."""
        x, y, yaw = pose
        dx, dy = self.but[0] - x, self.but[1] - y
        dist = math.hypot(dx, dy)
        if dist < self.tol:
            self.arrive = True
            return (0.0, 0.0, 0.0), True

        err = _norm_deg(math.degrees(math.atan2(dy, dx)) - yaw)   # > 0 : but a droite
        vmax = config.VITESSE_AVANT
        vlat = config.VITESSE_LATERALE
        seuil = config.SEUIL_OBSTACLE_CM

        # un mur ou un objet situe APRES le but (but devant nous) n'est pas un obstacle
        if abs(err) < 30.0 and avant_cm / 100.0 + config.ROBOT_RAYON_M > dist + 0.05:
            avant_cm = config.DISTANCE_INVALIDE_CM

        v = min(vmax, 0.5 * dist + 0.05)                  # ralentit en arrivant
        avance = v * max(0.0, math.cos(math.radians(err)))
        vz = _clamp(config.KP_ROT_NAV * err, config.VITESSE_ROT_MAX)
        vx, vy = avance, 0.0

        self._t += dt
        self._t_total += dt

        # blocage : on commande de l'avance mais la position ne change pas
        # (obstacle que les capteurs n'ont pas vu, par exemple un coin)
        if self._prev is not None and self._v_prec > 0.05 and \
                math.hypot(x - self._prev[0], y - self._prev[1]) < 0.2 * self._v_prec * dt:
            self._t_bloque += dt
        else:
            self._t_bloque = 0.0
        self._prev = (x, y)
        if self._t_bloque > 1.0 and self.etat in ("LIBRE", "AVANCE"):
            self.cote = -self.cote if self.cote else 1
            self._yaw_ref = yaw
            self._t_bloque = 0.0
            self._changer("STRAFE")

        if self.etat == "LIBRE":
            # on ralentit a l'approche d'un obstacle
            vx *= max(0.5, min(1.0, (avant_cm - seuil) / 60.0 + 0.3))
            if avant_cm < seuil:
                # on contourne du cote du but si la place le permet, sinon du cote le plus libre
                lateral = dist * math.sin(math.radians(err))        # > 0 : but a droite
                libre = 1 if droite_cm >= gauche_cm else -1
                prefere = 1 if lateral > 0 else -1
                place = droite_cm if prefere == 1 else gauche_cm
                self.cote = prefere if (abs(lateral) > 0.25 and place > 40) else libre
                self._yaw_ref = yaw
                self._changer("STRAFE")
            else:
                # on s'ecarte des obstacles lateraux et on ne tourne pas vers eux
                if gauche_cm < DIST_REPULSION_CM:
                    vy += vlat * (DIST_REPULSION_CM - gauche_cm) / DIST_REPULSION_CM
                    vz = max(0.0, vz)
                if droite_cm < DIST_REPULSION_CM:
                    vy -= vlat * (DIST_REPULSION_CM - droite_cm) / DIST_REPULSION_CM
                    vz = min(0.0, vz)

        if self.etat in ("STRAFE", "MARGE"):
            cote_libre = droite_cm if self.cote == 1 else gauche_cm
            if cote_libre < 12:                            # plus de place de ce cote : on change
                self.cote = -self.cote
                self._changer("STRAFE")
        cap = _clamp(config.KP_ROT_NAV * _norm_deg(self._yaw_ref - yaw), config.VITESSE_ROT_MAX)
        if self.etat == "STRAFE":
            vx, vz = 0.0, cap
            vy = self.cote * vlat
            if avant_cm >= seuil + 10:
                self._changer("MARGE")
        elif self.etat == "MARGE":
            vx, vz = 0.0, cap
            vy = self.cote * vlat
            if avant_cm < seuil:
                self._changer("STRAFE")
            elif self._t >= T_MARGE_S:
                self._changer("AVANCE")
        elif self.etat == "AVANCE":
            vz = cap
            vx = 0.6 * avance if avance > 0 else 0.6 * vmax
            cote_obstacle = gauche_cm if self.cote == 1 else droite_cm
            if avant_cm < seuil:
                self._changer("STRAFE")
                vx, vy = 0.0, self.cote * vlat
            else:
                if cote_obstacle < 25:
                    vy = self.cote * 0.5 * vlat           # on s'ecarte du mur
                if cote_obstacle < DIST_LATERALE_CM:
                    self._vu = True                # l'obstacle est a cote de nous
                    self._t_libre = 0.0
                elif self._vu:
                    self._t_libre += dt            # on l'a depasse
                if self._t_libre >= T_DEGAGE_S or self._t > 8.0:
                    self._changer("LIBRE")

        if self.etat != "LIBRE" and self._t > 15.0:       # bloque : on essaie l'autre cote
            self.cote = -self.cote
            self._changer("STRAFE")

        self._v_prec = math.hypot(vx, vy)
        return (vx, vy, vz), False

    def _changer(self, etat):
        self.etat = etat
        self._t = 0.0
        self._t_libre = 0.0
        self._vu = False
'''
REMPLACES["behaviors/follow_person.py"] = r'''"""Suivi de personne (Mohamed) : rotation pour centrer + avance/recul pour garder la distance.

step() ne pilote pas le robot : il renvoie une commande (vx, vy, vz), ou None si la cible est perdue.
"""
import config
from control.pid import PID


class Suiveur:
    def __init__(self):
        self.pid_rot = PID(config.KP_ROT, zone_morte=15.0, sortie_max=config.VITESSE_ROT_MAX)
        self.pid_dist = PID(config.KP_DIST, zone_morte=10.0, sortie_max=config.VITESSE_MAX)

    def reset(self):
        self.pid_rot.reset()
        self.pid_dist.reset()

    def step(self, boite, dt=0.1):
        """boite = (x1, y1, x2, y2, ...) de la personne dans l'image, ou None."""
        if boite is None:
            self.reset()
            return None
        x1, _y1, x2, _y2 = boite[:4]
        err_x = (x1 + x2) / 2.0 - config.CAMERA_LARGEUR_PX / 2.0     # > 0 : cible a droite
        err_d = config.LARGEUR_CIBLE_PX - (x2 - x1)                  # > 0 : cible trop loin
        vz = self.pid_rot.step(err_x, dt)
        vx = self.pid_dist.step(err_d, dt)
        return (vx, 0.0, vz)
'''
REMPLACES["behaviors/pick_place.py"] = r'''"""Approche, saisie, depose (Bilal). Ne pilote pas le robot : renvoie des commandes.

- Approcheur.step(objet, avant_cm) -> commande (vx, vy, vz) + indicateur "pret a saisir"
- saisir(arm) / deposer(arm) : sequences du bras (voir control/arm_ctrl.py)

Convention SDK : vz > 0 = rotation vers la droite (horaire).
"""
import config
from control.pid import PID


class Approcheur:
    def __init__(self):
        self.pid_rot = PID(config.KP_ROT, zone_morte=8.0, sortie_max=config.VITESSE_ROT_MAX)

    def reset(self):
        self.pid_rot.reset()

    def step(self, objet, avant_cm, dt=0.1):
        """objet = (cx, taille) en pixels (ou None si perdu). Retourne ((vx, vy, vz), pret)."""
        if objet is None:
            self.reset()
            return None, False
        cx, _taille = objet
        err_x = cx - config.CAMERA_LARGEUR_PX / 2.0            # > 0 : objet a droite
        vz = self.pid_rot.step(err_x, dt)
        cible, tol = config.DIST_PRISE_CM, config.TOL_DIST_PRISE_CM
        if abs(err_x) > 60:                                    # on s'oriente d'abord
            return (0.0, 0.0, vz), False
        ecart = avant_cm - cible                               # > 0 : trop loin
        if abs(err_x) <= 12 and abs(ecart) <= tol:
            return (0.0, 0.0, 0.0), True
        vx = max(-0.08, min(0.25, 0.006 * ecart))
        if 0 < abs(vx) < 0.04:
            vx = 0.04 if vx > 0 else -0.04
        return (vx, 0.0, vz), False


def saisir(arm):
    arm.prendre()
    return arm.objet_saisi()


def deposer(arm):
    arm.deposer()
'''
REMPLACES["control/arm_ctrl.py"] = r'''"""Bras et pince (Bilal) : positions repos / prise / depose (config.POS_BRAS, en mm).

Marche avec le vrai robot ET avec le faux robot du simulateur.
"""
import time

import config


class Arm:
    def __init__(self, ep):
        self.ep = ep

    def _attendre(self, action=None):
        if action is not None and hasattr(action, "wait_for_completed"):
            action.wait_for_completed()
        if config.MODE == "real":
            time.sleep(1.0)          # la pince n'a pas d'action a attendre

    def aller(self, nom):
        """Deplace le bras vers config.POS_BRAS[nom] ('repos', 'prise' ou 'depose')."""
        x, y = config.POS_BRAS[nom]
        self._attendre(self.ep.robotic_arm.moveto(x=x, y=y))

    def repos(self):
        self.aller("repos")

    def ouvrir(self):
        self._attendre(self.ep.gripper.open(power=config.PUISSANCE_PINCE))

    def fermer(self):
        self._attendre(self.ep.gripper.close(power=config.PUISSANCE_PINCE))

    def prendre(self):
        """Ouvrir la pince, aller en position de prise, fermer, remonter au repos."""
        self.ouvrir()
        self.aller("prise")
        self.fermer()
        self.aller("repos")

    def deposer(self):
        """Aller en position de depose, ouvrir, revenir au repos."""
        self.aller("depose")
        self.ouvrir()
        self.aller("repos")

    def objet_saisi(self):
        """True si la pince tient l'objet. Dans le simulateur : info exacte.
        Sur le vrai robot : A IMPLEMENTER avec l'etat de la pince (gripper.sub_status)."""
        verif = getattr(self.ep.gripper, "objet_saisi", None)
        return verif() if verif is not None else True
'''
REMPLACES["mission/state_machine.py"] = r'''"""Machine a etats de la mission (Pierre-Olivier).

INIT -> CHERCHER -> APPROCHER -> SAISIR -> NAVIGUER -> DEPOSER -> FIN   (+ ERREUR)

Mission.step(capteurs, dt) est appelee a chaque tour de boucle et renvoie une commande
(vx, vy, vz). La mission decide quels comportements sont actifs a chaque etat ; l'arbitre
(mission/arbiter.py) garde la priorite a la securite.
  but = None  -> mission "prise d'objet" seulement (S3) : on depose sur place
  but = (x,y) -> mission complete : on va au but (odometrie) puis on depose
"""
from collections import namedtuple

import config
from behaviors.navigate_a_to_b import Navigateur
from behaviors.pick_place import Approcheur
from mission.arbiter import ARRET, choisir

Capteurs = namedtuple("Capteurs", "avant gauche droite pose objet")

T_MAX = {"CHERCHER": 40.0, "APPROCHER": 40.0, "NAVIGUER": 120.0}
TENTATIVES_MAX = 3


class Mission:
    def __init__(self, arm, but=None):
        self.arm = arm
        self.but = but
        self.etat = "INIT"
        self.message = ""
        self._t = 0.0
        self._rot = 0.0
        self._tentatives = 0
        self._appro = Approcheur()
        self._nav = None

    @property
    def terminee(self):
        return self.etat in ("FIN", "ERREUR")

    def _aller(self, etat, message=""):
        self.etat = etat
        self.message = message
        self._t = 0.0

    def step(self, c, dt=0.1):
        """c : Capteurs(avant, gauche, droite, pose, objet). Retourne (vx, vy, vz)."""
        self._t += dt
        cmd = ARRET
        if self.etat in T_MAX and self._t > T_MAX[self.etat]:
            self._aller("ERREUR", "temps depasse en " + self.etat)
            return ARRET

        if self.etat == "INIT":
            self.arm.repos()
            self._aller("CHERCHER", "on cherche l'objet")

        elif self.etat == "CHERCHER":
            if c.objet is not None:
                self._appro.reset()
                self._aller("APPROCHER", "objet vu")
            else:
                self._rot += config.VITESSE_RECHERCHE_DEG * dt
                if self._rot > 400.0:
                    self._aller("ERREUR", "objet introuvable")
                else:
                    cmd = (0.0, 0.0, config.VITESSE_RECHERCHE_DEG)

        elif self.etat == "APPROCHER":
            # ici on N'utilise PAS l'evitement : l'objet est justement la chose dont on s'approche
            cmd_a, pret = self._appro.step(c.objet, c.avant, dt)
            if cmd_a is None:
                self._aller("CHERCHER", "objet perdu")
                self._rot = 0.0
            elif pret:
                self._aller("SAISIR", "pret a saisir")
            else:
                cmd = cmd_a

        elif self.etat == "SAISIR":
            self._tentatives += 1
            saisi = self.arm.prendre() or self.arm.objet_saisi()
            if self.arm.objet_saisi():
                if self.but is None:
                    self._aller("DEPOSER", "objet saisi, depose sur place")
                else:
                    self._nav = Navigateur(self.but)
                    self._aller("NAVIGUER", "objet saisi, direction B")
            elif self._tentatives < TENTATIVES_MAX:
                self._appro.reset()
                self._aller("APPROCHER", "saisie ratee, on recommence")
            else:
                self._aller("ERREUR", "saisie impossible")

        elif self.etat == "NAVIGUER":
            cmd, arrive = self._nav.step(c.pose, c.avant, c.gauche, c.droite, dt)
            if arrive:
                self._aller("DEPOSER", "arrive en B")
                cmd = ARRET

        elif self.etat == "DEPOSER":
            self.arm.deposer()
            self._aller("FIN", "objet depose")

        # la securite reste prioritaire (sauf pendant l'approche de l'objet)
        securite = ARRET if (self.etat not in ("APPROCHER", "CHERCHER") and c.avant < 8.0) else None
        return choisir([securite, cmd])
'''
CONFIG_BLOC_1 = r'''
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
'''
CONFIG_BLOC_2 = r'''
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
'''


def ecrire(chemin, contenu, remplacer):
    existe = os.path.exists(chemin)
    if existe and not remplacer:
        print("  existe     ", chemin)
        return
    dossier = os.path.dirname(chemin)
    if dossier:
        os.makedirs(dossier, exist_ok=True)
    with open(chemin, "w", encoding="utf-8", newline="\n") as f:
        f.write(contenu)
    print("  mis a jour " if existe else "  cree       ", chemin)


def main():
    print("Ajout du simulateur 2D dans", os.getcwd())
    if not os.path.exists("config.py") or not os.path.isdir("core"):
        print("ERREUR : lance ce script a la racine du depot (la ou se trouve config.py).")
        return 1
    for chemin in sorted(NOUVEAUX):
        ecrire(chemin, NOUVEAUX[chemin], True)
    for chemin in sorted(REMPLACES):
        ecrire(chemin, REMPLACES[chemin], True)

    for marqueur, bloc, nom in (("TOF_ANGLES_DEG", CONFIG_BLOC_1, "simulateur 2D"),
                                ("DIST_PRISE_CM", CONFIG_BLOC_2, "manipulation")):
        with open("config.py", encoding="utf-8") as f:
            cfg = f.read()
        if marqueur not in cfg:
            with open("config.py", "a", encoding="utf-8", newline="\n") as f:
                if not cfg.endswith("\n"):
                    f.write("\n")
                f.write(bloc)
            print("  complete    config.py (bloc " + nom + ")")
        else:
            print("  existe      config.py (bloc " + nom + " deja present)")

    print("\nTermine. Etapes suivantes :  python -m pytest   puis   python main.py sim")
    return 0


if __name__ == "__main__":
    sys.exit(main())
