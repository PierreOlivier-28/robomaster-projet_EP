"""Machine a etats de la mission (Pierre-Olivier).

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
