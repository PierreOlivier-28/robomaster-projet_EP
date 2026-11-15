# Projet DJI RoboMaster EP - ESIGELEC 2026/2027

Robot mobile à roues mecanum (DJI RoboMaster EP), équipe de 4 : **Pierre-Olivier** (intégration, mission), **Lenaic** (évitement, navigation), **Mohamed** (tracking de personne), **Bilal** (prise d'objet).

Quatre scénarios :

| Scénario | Objectif |
|---|---|
| **S1** | Aller d'un point A à un point B en évitant les obstacles (décalage latéral) |
| **S2** | Suivre une personne avec la caméra |
| **S3** | Prendre un objet et le déposer |
| **Mission** | Prendre l'objet, aller en B en évitant les obstacles, le déposer |

> **Principe du projet : on développe chez soi, dans un simulateur 2D, et on ne teste sur le vrai robot qu'au labo, pour régler.** Il n'y a qu'un seul robot : tout ce qui peut être vérifié sans lui doit l'être sans lui.

**Sommaire**
1. [Premier jour : installer son poste](#1-premier-jour--installer-son-poste)
2. [Utiliser Git et GitHub sans rien casser](#2-utiliser-git-et-github-sans-rien-casser)
3. [Organisation du code : où mettre quoi](#3-organisation-du-code--où-mettre-quoi)
4. [Le simulateur 2D](#4-le-simulateur-2d)
5. [Tester son code](#5-tester-son-code)
6. [Qui fait quoi et situations à tester](#6-qui-fait-quoi-et-situations-à-tester)
7. [Passer au vrai robot (labo)](#7-passer-au-vrai-robot-labo)
8. [Règles d'équipe](#8-règles-déquipe)
9. [Problèmes fréquents](#9-problèmes-fréquents)

---

## 1. Premier jour : installer son poste

À faire **une seule fois**, **avant** de se connecter au Wi-Fi du robot : le mode AP du robot n'a pas d'accès Internet, donc aucune installation n'est possible ensuite.

### 1.1 Installer les outils
- **Git** : <https://git-scm.com/download/win> (tout laisser par défaut).
- **Python 3.8** (obligatoire : le SDK `robomaster` ne s'installe pas avec Python 3.9 et plus).
- **VS Code** (conseillé).
- Un compte **GitHub**, puis donner son pseudo à Pierre-Olivier pour être invité sur le dépôt (accepter l'invitation reçue par e-mail).

Dans PowerShell, vérifier :
```powershell
git --version
py -3.8 --version        # doit afficher Python 3.8.x
```

### 1.2 Se présenter à Git (une fois)
```powershell
git config --global user.name  "Prénom Nom"
git config --global user.email "ton.email@exemple.com"     # le même que sur GitHub
```

### 1.3 Récupérer le projet
Se placer dans le dossier où ranger ses projets, puis :
```powershell
git clone https://github.com/PierreOlivier-28/robomaster-projet_EP.git
cd robomaster-projet_EP
```
La première fois, GitHub demande de se connecter dans le navigateur : accepter.

### 1.4 Créer son environnement Python
Dans le dossier du dépôt :
```powershell
py -3.8 -m venv venv
venv\Scripts\activate                 # le prompt affiche maintenant (venv)
python -m pip install --upgrade pip
pip install -r requirements.txt
```
Si PowerShell refuse d'activer le venv : `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned`, puis recommencer.
À chaque nouvelle fenêtre PowerShell : se placer dans le dépôt et refaire `venv\Scripts\activate`.

### 1.5 Vérifier que tout marche
```powershell
python -m pytest            # doit afficher: 34 passed, 1 deselected
python main.py sim          # ouvre le simulateur 2D
```

> Mohamed : pour YOLO (`torch`, `ultralytics`), installer et télécharger le modèle **maintenant**, à la maison (décommenter les lignes dans `requirements.txt`, ranger le modèle dans `models/`).

---

## 2. Utiliser Git et GitHub sans rien casser

**Idée générale.** Le dépôt GitHub est la version de référence. Chacun en a une **copie sur son PC**. On ne modifie jamais la branche `main` directement : on travaille sur **sa propre branche**, puis on propose ses changements par une **Pull Request** (PR) que quelqu'un relit avant de les fusionner.

```
 GitHub (main) ──pull──► mon PC (main) ──checkout -b──► ma branche
                                                          │ je modifie, je teste
 GitHub (main) ◄──merge PR── GitHub (ma branche) ◄──push──┘
```

### 2.1 Le cycle complet (à suivre à chaque fois)

**① Partir de `main` à jour**
```powershell
git checkout main
git pull
```

**② Créer SA branche** (un sujet = une branche ; nom clair : `feat/tracking-3m`, `fix/evitement-coin`, `docs/readme`)
```powershell
git checkout -b feat/mon-sujet
```

**③ Travailler** : modifier le code, le lancer dans le simulateur, écrire un test (voir parties 4 et 5).

**④ Vérifier avant d'envoyer**
```powershell
python -m pytest            # TOUT doit être vert
git status                  # liste ce qui a changé : rien d'inconnu ? (pas de venv, pas de gros fichiers)
```

**⑤ Enregistrer (commit)**
```powershell
git add behaviors/follow_person.py tests/unit/test_tracking.py     # ses fichiers, explicitement
git commit -m "Tracking : suivi d'une personne a 3 m"
```
Un message de commit dit **ce qu'on a fait**, pas « modif » ni « test ».

**⑥ Envoyer sur GitHub**
```powershell
git push -u origin feat/mon-sujet
```
(la 2ᵉ fois sur la même branche, `git push` suffit).

**⑦ Ouvrir la Pull Request** : sur GitHub, ouvrir le dépôt, cliquer sur le bandeau jaune **Compare & pull request**, remplir la description et la checklist, choisir un relecteur à droite (*Reviewers*), puis **Create pull request**. Si le code doit être essayé sur le robot, ajouter le label `a-tester-robot`.

**⑧ Relecture, puis merge.** Le relecteur lit, commente ou approuve (*Approve*). Une fois approuvée : **Merge pull request** → **Confirm merge** → **Delete branch**.

**⑨ Nettoyer chez soi**
```powershell
git checkout main
git pull
git branch -d feat/mon-sujet
```
Puis on recommence au ① pour le sujet suivant.

### 2.2 Récupérer le travail des autres pendant qu'on travaille
Si un coéquipier a fusionné une PR pendant que vous travaillez :
```powershell
git fetch origin
git merge origin/main        # ramène les nouveautés dans sa branche
python -m pytest
```
Si Git signale un **conflit**, ne pas paniquer : ouvrir le fichier, repérer les lignes entre `<<<<<<<` et `>>>>>>>`, garder la bonne version (en parlant avec l'auteur), supprimer les marqueurs, puis `git add` du fichier et `git commit`.

### 2.3 Les règles pour ne rien casser
- ❌ **Jamais** de travail directement sur `main`. Avant de modifier quoi que ce soit : `git branch` doit afficher une branche `feat/...` (l'étoile `*`).
- ❌ **Jamais** de `git push --force`, ni de `git reset --hard` sans en parler.
- ✅ **Toujours** `git pull` sur `main` avant de créer une branche.
- ✅ **Toujours** `python -m pytest` vert avant un push.
- ✅ Ne modifier que **ses fichiers** (voir `.github/CODEOWNERS` et la partie 6). Pour toucher au fichier d'un autre : le prévenir, ou lui demander.
- ✅ Les réglages (vitesses, seuils, ports) se font **uniquement** dans `config.py`.
- ✅ Petites PR, fréquentes : une PR de 30 lignes se relit en 5 minutes, une PR de 800 lignes ne se relit pas.

### 2.4 Commandes utiles
| Je veux... | Commande |
|---|---|
| Savoir sur quelle branche je suis | `git branch` |
| Voir ce que j'ai modifié | `git status` puis `git diff` |
| Annuler mes modifications d'**un** fichier (pas encore commité) | `git restore chemin/du/fichier.py` |
| Voir l'historique | `git log --oneline -10` |
| Changer de branche | `git checkout nom-de-la-branche` |
| Supprimer une branche locale déjà fusionnée | `git branch -d nom` |

### 2.5 Quand tout est cassé
J'ai commité sur `main` par erreur (avant le push) :
```powershell
git branch feat/mon-sujet          # garde mon commit sur une nouvelle branche
git reset --hard origin/main       # remet main local comme GitHub
git checkout feat/mon-sujet
```
En cas de doute, **ne rien forcer** : copier le dossier du dépôt ailleurs par sécurité, puis demander à Pierre-Olivier.

> Les avertissements « LF will be replaced by CRLF » sous Windows sont normaux et sans danger.

---

## 3. Organisation du code : où mettre quoi

```
robomaster-projet_EP/
├── main.py                   point d'entrée : python main.py s1|s2|s3|mission|sim [--real]
├── config.py                 TOUS les réglages (vitesses, seuils, ports capteurs, bras...)
├── scenarios/                lancer un scénario sur le VRAI robot (s1, s2, s3, s4_mission_complete)
├── mission/                  enchaîner les étapes : state_machine.py, arbiter.py (priorités)
├── behaviors/                DÉCIDER quoi faire : avoid_obstacles, navigate_a_to_b, follow_person, pick_place
├── perception/               VOIR : detector (YOLO), tracker, object_locator
├── control/                  AGIR : chassis_ctrl (roues), arm_ctrl (bras/pince), pid
├── core/                     BASE : robot_io, sensors, odometry, camera, safety, logger
├── sim/                      simulateur 2D et faux robot (ne pas casser)
├── tests/unit/               tests sans robot (pytest)
├── tests/robot/              tests avec robot (pytest -m robot)
├── tools/                    outils (calibration du bras, rejeu vidéo)
├── models/  data/  logs/     modèles YOLO, données, journaux (non versionnés)
└── docs/
```

**Les dépendances vont vers le bas uniquement** : `scenarios → mission → behaviors → perception / control → core`. Un fichier de `behaviors/` n'importe jamais un fichier de `scenarios/`.

### Où ajouter mon code ?
| Je veux... | Je mets le code dans | Mon test va dans |
|---|---|---|
| Décider quoi faire (suivre, éviter, saisir) | `behaviors/` | `tests/unit/` |
| Voir (caméra, YOLO, position de l'objet) | `perception/` | `tests/unit/` (avec le faux détecteur) |
| Agir (roues, bras, pince) | `control/` | `tests/unit/` |
| Enchaîner les étapes d'une mission | `mission/` | `tests/unit/test_mission.py` |
| Lancer sur le vrai robot | `scenarios/` | `tests/robot/` |
| Changer une vitesse, un seuil, un port | `config.py` | - |
| Ajouter une situation au simulateur | `sim/world2d.py` | `tests/unit/test_world2d.py` |

### La règle la plus importante
Un comportement (`behaviors/`) est une **fonction qui renvoie une commande `(vx, vy, vz)`** ; **il ne pilote jamais le robot lui-même**. C'est la boucle (celle du simulateur ou celle d'un scénario) qui envoie la commande. C'est ce qui permet d'utiliser **exactement le même code** dans le simulateur et sur le vrai robot.

Convention du SDK : `vx` > 0 avance (m/s), `vy` > 0 va vers la **droite** (m/s), `vz` > 0 tourne vers la **droite**, sens horaire (°/s).

Autre règle : un module réel doit renvoyer **le même format que son faux**. Exemple : `perception/detector.py` renvoie une liste de `(x1, y1, x2, y2, classe, confiance)`, comme `sim/fake_detector.py`. Ainsi, passer du faux au vrai ne change rien au comportement.

---

## 4. Le simulateur 2D

Une vue **de dessus** (comme turtlesim) d'une arène de 4 m × 3 m, avec le robot, des obstacles, une personne, un objet et une zone de dépôt. Les comportements utilisés sont **exactement ceux du projet**, branchés sur un faux robot. Aucune dépendance supplémentaire : OpenCV et numpy suffisent.

```powershell
python main.py sim
```

### Ce qu'on voit
- le robot (rouge s'il touche quelque chose), son champ de caméra, sa trace ;
- les 4 rayons des capteurs de distance ToF (rouges quand l'obstacle est à moins de 50 cm) ;
- le point de départ **A**, le but **B**, les obstacles, la personne, l'objet, la zone de dépôt ;
- à droite : le mode, **l'état du comportement** (ex. « obstacle : décalage vers la droite »), la commande envoyée, les valeurs des capteurs, l'odométrie, l'état du bras et de la pince, et la **vue caméra** simulée.

### Les modes (touches du clavier, fenêtre active)
| Touche | Mode | Ce qui se passe |
|---|---|---|
| `1` | Évitement A→B | le robot va de A vers B ; devant un obstacle (< 50 cm) il se **décale sur le côté** par translation, longe l'obstacle puis reprend le cap sur B |
| `2` | Tracking | une personne marche devant lui, il la suit (sans obstacles, pour tester le suivi seul) |
| `3` | Prise d'objet | il cherche l'objet, s'approche, le saisit |
| `4` | Mission complète | il prend l'objet, va en B en évitant, le dépose dans la zone |
| `m` | Manuel | `w a s d` (déplacements), `q e` (rotation) |

Autres touches : `r` remet à zéro, `espace` pause, `p` personne qui marche / s'arrête, `ESC` ou `x` quitte.

### La souris
| Action | Effet |
|---|---|
| Clic gauche (mode 1) | change le but B |
| Clic gauche sur la personne, glisser (mode 2) | déplace la personne (elle s'arrête de marcher) |
| Clic gauche (modes 3 et 4) | place l'objet à cet endroit |
| Clic droit | ajoute un obstacle (ou le retire s'il y en a déjà un) |

### Ce que la simulation prouve, et ne prouve pas
Elle **valide la logique** (le robot fait-il la bonne chose dans cette situation ?). Elle **ne valide pas les réglages** : sur le vrai robot il y a du bruit, des glissements, des délais, une vraie caméra. Les gains et seuils se terminent au labo.

---

## 5. Tester son code

Deux niveaux, **dans cet ordre** :

1. **À l'œil** : `python main.py sim`, on reproduit la situation à la souris (voir partie 6).
2. **Automatique** : on écrit la même situation dans un test `pytest`, **sans fenêtre**. Il restera dans le dépôt et protégera le code des régressions.

```powershell
python -m pytest                                  # tous les tests sans robot
python -m pytest tests/unit/test_tracking.py      # un seul fichier
python -m pytest -k tracking                      # les tests dont le nom contient "tracking"
python -m pytest -m robot                         # tests AVEC robot (au labo seulement)
```

### Modèle de test (tracking : personne à 3 m, puis cachée derrière un obstacle)
À placer dans `tests/unit/test_tracking.py`. La simulation tourne par pas de 0,05 s, sans fenêtre.

```python
import math

from behaviors.follow_person import Suiveur
from mission.arbiter import ARRET, choisir
from sim.fake_detector import FakeDetector
from sim.world2d import World


def _suivre(w, duree_s=30.0):
    """Fait tourner la simulation sans fenetre. Retourne (distance finale, pas sans cible)."""
    det, suiveur = FakeDetector(w), Suiveur()
    perdus = 0
    for _ in range(int(duree_s / 0.05)):
        d = det.detect()
        avant = w.tof_cm()[0]
        suivi = suiveur.step(d[0][:4] if d else None, 0.05)
        perdus += 0 if d else 1
        cmd = choisir([ARRET if avant < 25 else None, suivi])
        w.set_cmd(*cmd, 1.0)
        w.step(0.05)
    x, y, _ = w.pose
    return math.hypot(w.person[0] - x, w.person[1] - y), perdus


def test_tracking_personne_a_3_metres():
    w = World()
    w.obstacles = []
    w.set_person(w.pose[0] + 3.0, w.pose[1])          # immobile, 3 m devant
    w.person_marche = False
    distance, perdus = _suivre(w)
    assert 0.8 < distance < 1.4                       # il s'est rapproché (cible ~1 m)
    assert perdus == 0 and not w.collision


def test_tracking_personne_cachee_derriere_un_obstacle():
    w = World()
    w.obstacles = []
    w.ajouter_obstacle(w.pose[0] + 1.5, w.pose[1], 0.5)   # boîte entre le robot et la personne
    w.set_person(w.pose[0] + 3.0, w.pose[1])
    w.person_marche = False
    distance, perdus = _suivre(w, 10.0)
    assert perdus > 0                                  # la cible n'est pas visible
    assert not w.collision                             # et le robot ne fonce pas dans la boîte
```

Autres modèles à copier : `tests/unit/test_navigation.py` (évitement, navigation), `test_manipulation.py` (prise d'objet), `test_mission.py` (mission complète), `test_world2d.py` (simulateur).

**Un bon test** décrit **une situation** (où sont le robot, la personne, les obstacles) et vérifie **un résultat mesurable** (distance finale, absence de collision, état atteint). Il ne dépend pas d'une fenêtre ni du hasard.

---

## 6. Qui fait quoi et situations à tester

Légende : **OK** = fait et testé en simulation · **À FAIRE** = reste à écrire · **À RÉGLER** = existe, à calibrer sur le robot.

### État d'avancement global
| Brique | Fichier | État | Responsable |
|---|---|---|---|
| Évitement A→B (décalage latéral) | `behaviors/navigate_a_to_b.py` | OK, à régler | Lenaic |
| Évitement simple (S1, sans but) | `behaviors/avoid_obstacles.py` | OK | Lenaic |
| Suivi de personne (asservissement) | `behaviors/follow_person.py` | OK, à régler | Mohamed |
| Détecteur de personne réel (YOLO) | `perception/detector.py` | **À FAIRE** (squelette) | Mohamed |
| Tracker avec identifiant | `perception/tracker.py` | **À FAIRE** (squelette) | Mohamed |
| Position de l'objet dans l'image | `perception/object_locator.py` | **À FAIRE** (squelette) | Bilal |
| Approche, saisie, dépose | `behaviors/pick_place.py` | OK, à régler | Bilal |
| Bras / pince, positions réelles | `control/arm_ctrl.py`, `POS_BRAS` | À RÉGLER | Bilal |
| Outil de calibration du bras | `tools/calibration_bras.py` | **À FAIRE** | Bilal |
| Machine d'états de la mission | `mission/state_machine.py` | OK | Pierre-Olivier |
| Arbitre de priorités | `mission/arbiter.py` | OK | Pierre-Olivier |
| Scénarios réels S2, S3, mission | `scenarios/s2…`, `s3…`, `s4…` | **À FAIRE** (squelettes) | chacun le sien |
| Simulateur 2D | `sim/` | OK | tous (ne pas casser) |

> Aujourd'hui, `python main.py s2 --real` (et `s3`, `mission`) **ne fait rien** : les scénarios réels sont des squelettes. Tout ce qui fonctionne, fonctionne dans le simulateur. Pour chaque scénario réel, recopier la boucle du mode correspondant de `sim/demo.py` (lire les capteurs, appeler le comportement, envoyer la commande) en remplaçant les faux éléments par les vrais.

### Mohamed : tracking (`behaviors/follow_person.py`, `perception/`)

| # | Situation | Comment la créer dans le simulateur | Résultat attendu | État |
|---|---|---|---|---|
| T1 | Personne qui marche | touche `2` | le robot la garde dans l'image, à ~1 m | OK |
| T2 | Personne immobile à **3 m** | touche `2`, `p` pour l'arrêter, la glisser à ~3 m devant le robot | il avance et s'arrête à ~1 m | OK (1,06 m, 0 collision) |
| T3 | Personne décalée sur le côté | la glisser en diagonale | il tourne, la centre, avance | OK |
| T4 | Personne **cachée derrière un obstacle** | touche `2`, clic droit pour poser un obstacle entre eux | cible perdue : le robot **s'arrête**, sans foncer dans l'obstacle | OK (comportement voulu) |
| T5 | La personne sort du champ puis revient | la glisser hors du champ, puis la ramener | arrêt, puis reprise du suivi | à vérifier |
| T6 | **Suivre ET contourner** un obstacle | obstacle sur le trajet, personne encore visible | le robot se décale, puis reprend le suivi | **À FAIRE** : seul un arrêt d'urgence à 25 cm existe |

**T6 est le vrai travail de Mohamed en simulation** : dans `mission/arbiter.py`, `choisir([...])` doit recevoir, par ordre de priorité, l'évitement (décalage latéral) puis le suivi. À décider avec Lenaic.

Pour le vrai robot : `perception/detector.py` (YOLO, modèle chargé **une seule fois**), puis `perception/tracker.py` (garder la même personne), puis `scenarios/s2_tracking.py` (boucle du mode `2` de `sim/demo.py`, `FakeDetector` remplacé par `Detector`).

### Lenaic : évitement et navigation (`behaviors/avoid_obstacles.py`, `behaviors/navigate_a_to_b.py`)

| # | Situation | Comment la créer | Résultat attendu | État |
|---|---|---|---|---|
| E1 | Obstacle sur la ligne A→B | touche `1` (2 obstacles par défaut) | décalage latéral, il longe l'obstacle, arrive en B | OK (~18 s, 0 collision) |
| E2 | B change en cours de route | clic gauche ailleurs | il repart vers le nouveau B | OK |
| E3 | Obstacle posé juste devant le robot | clic droit devant lui | décalage vers le côté le plus libre | OK |
| E4 | B collé à un mur | clic gauche près du bord | il arrive, sans prendre le mur pour un obstacle | OK |
| E5 | Poche / impasse entre deux obstacles | clic droit pour fermer un passage | il tente l'autre côté, ou signale l'échec | **LIMITE** : ~56 cas sur 60 réussis en positions aléatoires |
| E6 | Coin d'obstacle non vu par un capteur | obstacle de biais | pas de collision | détection de blocage présente, à surveiller |

À faire : traiter E5 et E6, puis `scenarios/s1_navigation.py` pour le vrai robot (avec `Navigateur`, comme le mode `1` de `sim/demo.py`), puis régler `SEUIL_OBSTACLE_CM`, `VITESSE_LATERALE`, `KP_ROT_NAV` dans `config.py`.

### Bilal : prise d'objet (`behaviors/pick_place.py`, `control/arm_ctrl.py`, `perception/object_locator.py`)

| # | Situation | Comment la créer | Résultat attendu | État |
|---|---|---|---|---|
| P1 | Objet devant, visible | touche `3`, clic gauche pour placer l'objet | recherche, approche, prise, état `FIN` | OK |
| P2 | Objet sur le côté | placer l'objet de biais | le robot tourne pour le chercher | OK |
| P3 | Objet derrière un obstacle ou hors de vue au départ | clic droit + clic gauche | message d'erreur clair, pas de blocage | **LIMITE** : ~12 positions sur 18 réussies |
| P4 | Prise ratée (objet non tenu) | - | retour à la recherche ou erreur | **À FAIRE** côté vrai robot |
| P5 | Mission complète | touche `4` | objet déposé dans la zone | OK |

Pour le vrai robot : `perception/object_locator.py` (renvoie `(cx, taille)` ou `None`, comme `sim/fake_object_locator.py`), calibrer `POS_BRAS` dans `config.py` avec `tools/calibration_bras.py`, **détecter la prise réelle** (`gripper.sub_status`) car `Arm.objet_saisi()` renvoie aujourd'hui toujours `True`, puis `scenarios/s3_manipulation.py`.

### Pierre-Olivier : intégration (`mission/`, `scenarios/s4_mission_complete.py`)
- relire les PR, garder `main` vert, fusionner ;
- écrire `scenarios/s4_mission_complete.py` (boucle du mode `4` de `sim/demo.py`) ;
- vérifier que les modules de Mohamed, Lenaic et Bilal s'enchaînent dans `mission/state_machine.py` ;
- au labo : vérifier `PORT_*` et le sens du yaw avant tout autre test.

---

## 7. Passer au vrai robot (labo)

**Étapes pour chaque fonction :**
1. **Simulation** : écrire le comportement, le voir dans le simulateur, ajouter un test.
2. **Remplacer le faux par le vrai** : `FakeDetector` → `Detector`, `FakeObjectLocator` → `object_locator`. Même format de sortie, donc le comportement ne change pas.
3. **Robot** : PR avec le label `a-tester-robot`, test au labo, réglage dans `config.py`, puis tag Git quand ça marche.

**Se connecter au robot.** Robot allumé en mode AP, PC connecté au Wi-Fi `RMEP-xxxxxx` (pas d'Internet pendant ce temps) :
```powershell
python main.py s1 --real
python -m pytest -m robot
```

**À vérifier EN PREMIER au labo (avant tout réglage) :**
- le numéro de chaque capteur ToF (`PORT_AVANT`, `PORT_DROITE`, `PORT_ARRIERE`, `PORT_GAUCHE` dans `config.py`) ;
- le sens et l'unité du `yaw` de l'odométrie (positif vers la droite, en degrés, dans le simulateur) ;
- les positions du bras (`POS_BRAS`) et la détection de prise de la pince.

**Après une séance robot qui marche** : `git tag robot-ok-AAAA-MM-JJ` puis `git push origin robot-ok-AAAA-MM-JJ`, et noter les essais dans le Plan de tests. Un tag `robot-ok-…` désigne une version **qui a marché sur le robot** : en cas de problème, on peut y revenir.

---

## 8. Règles d'équipe

- Jamais de push direct sur `main` : **branche + Pull Request + 1 relecture**.
- Chacun ne modifie que ses fichiers (voir `.github/CODEOWNERS`).
- `python -m pytest` vert avant chaque PR.
- Réglages uniquement dans `config.py`.
- Tout test sur robot est noté dans le Plan de tests ; tag Git après chaque séance robot réussie.
- Utilisation d'une IA : l'écrire dans le Registre IA.
- Une seule personne à la fois sur le robot : se coordonner pour les créneaux.

---

## 9. Problèmes fréquents

| Symptôme | Cause et solution |
|---|---|
| `pip install robomaster` échoue | Mauvaise version de Python : refaire le venv avec `py -3.8 -m venv venv` |
| `pytest` « non reconnu » | Le venv n'est pas activé : `venv\Scripts\activate` |
| `python main.py sim` : pas de fenêtre | Installer `opencv-python` dans le venv : `pip install -r requirements.txt` |
| PowerShell refuse `activate` | `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned` |
| `git push` refusé : `src refspec ... does not match any` | La branche n'existe pas : voir 2.5, ou `git checkout -b feat/mon-sujet` |
| `git push` refusé : `main` protégée | Normal : on pousse sa branche, pas `main` |
| `git push` refusé : le distant a avancé | `git fetch origin`, `git merge origin/main`, `python -m pytest`, `git push` |
| Conflit lors du merge | Voir 2.2 |
| `git branch -d` refuse (not fully merged) | La PR n'est pas encore fusionnée, ou `git pull` n'a pas été fait sur `main` |
| Le robot va du mauvais côté au labo | Vérifier `PORT_*` et le sens du yaw (partie 7) |
| Le simulateur passe, le robot échoue | Normal : la simulation valide la logique, pas les réglages. Régler dans `config.py` |
