"""Navigation A -> B avec evitement d'obstacles (Lenaic).

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
