"""Fenetre du simulateur 2D (OpenCV) : dessin + souris. Aucune logique de robot ici."""
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
