"""Monde 2D simule, vue de dessus. Aucune dependance graphique : testable sans fenetre.

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
