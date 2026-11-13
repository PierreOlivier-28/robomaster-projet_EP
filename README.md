# Projet DJI RoboMaster EP – ESIGELEC 2026/2027

Robotique mobile et perception, avec 3 scénarios :
- **S1** : navigation et évitement d'obstacles (Lénaïc)
- **S2** : tracking d'une personne (Mohamed)
- **S3** : manipulation d'objets avec le bras (Bilal)
- **Mission complète** : tout assembler (Pierre-Olivier)

Le code se développe **chez soi en simulation** (faux robot). On le teste sur le **vrai robot** pendant les séances au labo.

---

## 1. Installation (à faire UNE SEULE FOIS)

### 1.1 Installer les outils
1. **Git** : https://git-scm.com/download/win (garder toutes les options par défaut).
2. **Python 3.8** : https://www.python.org/downloads/release/python-3810/ (cocher « Add Python to PATH »). **Python 3.8 est obligatoire** : le SDK du robot ne marche pas avec 3.9 ou plus.
3. **VS Code** (recommandé) : https://code.visualstudio.com/
4. Un compte **GitHub** : https://github.com. Donner ton pseudo à Pierre-Olivier pour être invité au dépôt, puis **accepter l'invitation** (reçue par email).

### 1.2 Configurer Git (ouvrir PowerShell)
```powershell
git config --global user.name "Prénom Nom"
git config --global user.email "email_de_ton_compte_github@exemple.com"
git config --global init.defaultBranch main
```

### 1.3 Récupérer le projet
Choisis un dossier de travail (pas dans OneDrive si possible), puis :
```powershell
cd C:\chemin\vers\ton\dossier
git clone https://github.com/PierreOlivier-28/robomaster-projet_EP.git
cd robomaster-projet_EP
```
Une fenêtre de connexion GitHub s'ouvre : connecte-toi.

### 1.4 Créer l'environnement Python
```powershell
py -3.8 --version
py -3.8 -m venv venv
venv\Scripts\activate
python -m pip install --upgrade pip
pip install -r requirements.txt
```
- La première commande doit afficher `Python 3.8.x`.
- Si `activate` est refusé (« l'exécution de scripts est désactivée ») : `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned`, répondre `O`, puis refaire `venv\Scripts\activate`.
- Quand l'environnement est actif, `(venv)` s'affiche au début de la ligne. **À refaire à chaque nouveau terminal** : `venv\Scripts\activate`.

### 1.5 Vérifier que tout marche
```powershell
python -m pytest
python main.py s1
```
- `pytest` doit afficher **8 passed**.
- `main.py s1` doit afficher des lignes `front=... cm cmd=...` pendant 10 secondes (le faux robot avance, puis tourne quand un obstacle est proche).

Si ça marche, ton poste est prêt.

---

## 2. Vocabulaire Git en 1 minute

| Mot | Signification |
|---|---|
| **Dépôt (repo)** | Le dossier du projet, avec tout son historique |
| **Clone** | Copie du dépôt GitHub sur ton PC (une seule fois) |
| **Branche** | Une copie de travail à toi. Tu y fais tes modifications sans toucher au projet principal |
| **`main`** | La branche principale : **toujours du code qui marche**. On n'y travaille jamais directement |
| **Commit** | Une sauvegarde de tes modifications, avec un message |
| **Push** | Envoyer tes commits de ton PC vers GitHub |
| **Pull** | Récupérer les modifications des autres depuis GitHub |
| **Pull request (PR)** | Demande pour ajouter ta branche à `main`. Quelqu'un relit avant |

Le principe : **tu travailles sur ta branche, puis une PR relue ajoute ton travail à `main`**. Ainsi, rien ne casse le travail des autres.

---

## 3. Routine de travail (à chaque fois)

### Étape 1 : partir de la dernière version
```powershell
cd robomaster-projet_EP
venv\Scripts\activate
git checkout main
git pull
```

### Étape 2 : créer ta branche (une par tâche)
```powershell
git checkout -b s2/tracking-yolo
```
Nom de branche : `scenario/sujet`, par exemple `s1/evitement-tof`, `s3/pince`, `core/safety`. Pas d'espaces ni d'accents.

### Étape 3 : coder et tester
Modifie **tes fichiers** (voir section 5), puis lance :
```powershell
python -m pytest
```
**Tous les tests doivent passer** avant d'aller plus loin.

### Étape 4 : sauvegarder (commit)
```powershell
git status                              # voir ce qui a changé
git add perception/detector.py          # ajouter UNIQUEMENT tes fichiers
git commit -m "Détecteur YOLO : détection de personnes sur webcam"
```
- Un message clair qui dit ce que tu as fait.
- Fais de **petits commits** souvent (toutes les heures ou à chaque étape).
- Évite `git add .` sauf si `git status` montre que seuls tes fichiers ont changé.

### Étape 5 : envoyer sur GitHub (push)
La première fois pour cette branche :
```powershell
git push -u origin s2/tracking-yolo
```
Les fois suivantes : `git push`.

### Étape 6 : ouvrir la pull request
1. Sur GitHub, ouvre le dépôt. Un bandeau jaune **Compare & pull request** apparaît : clique dessus.
2. Remplis la description et la **checklist** (tests, robot, registre IA).
3. À droite, dans **Reviewers**, choisis un coéquipier (voir section 6).
4. Si ton code doit être testé sur le robot, ajoute le label **`à-tester-robot`**.
5. Clique sur **Create pull request**.

### Étape 7 : relire (si on te demande une relecture)
1. Ouvre la PR, onglet **Files changed**.
2. Lis le code. Pour commenter une ligne, clique sur le `+` bleu à gauche de la ligne.
3. En haut à droite : **Review changes** → **Approve** (ou **Request changes** s'il y a un problème) → **Submit review**.
4. Ce n'est pas à toi de merger si ce n'est pas ta PR : c'est l'auteur qui clique ensuite sur **Merge pull request**, quand tout est vert.

### Étape 8 : nettoyer après le merge
```powershell
git checkout main
git pull
git branch -d s2/tracking-yolo
```
Puis recommence à l'étape 1 pour la tâche suivante.

---

## 4. Règles d'or (pour ne rien casser)

**À FAIRE**
- `git pull` sur `main` **avant** de créer une branche.
- Travailler **uniquement sur une branche**, jamais sur `main`.
- Lancer `python -m pytest` avant chaque push.
- Faire de petits commits avec des messages clairs.
- Déclarer l'IA utilisée dans le Registre IA (fichier de gestion de projet).
- Demander dans le groupe avant de modifier le fichier de quelqu'un d'autre.

**À NE JAMAIS FAIRE**
- ❌ `git push --force` (efface le travail des autres).
- ❌ Commiter le dossier `venv/`, des poids YOLO (`.pt`), des gros fichiers ou le dossier `logs/`. Le `.gitignore` les bloque normalement.
- ❌ Modifier `config.py`, `core/robot_io.py` ou `main.py` sans prévenir Pierre-Olivier.
- ❌ Copier-coller du code d'une IA sans le comprendre : on doit pouvoir l'expliquer en soutenance.
- ❌ Fermer une PR ou supprimer une branche qui n'est pas la tienne.
- ❌ Changer l'interface (les noms et paramètres des fonctions) d'un module sans prévenir ceux qui l'utilisent.

---

## 5. Qui s'occupe de quoi

Chacun **ne modifie que ses fichiers**. Cela évite 95 % des conflits.

| Personne | Fichiers |
|---|---|
| **Lénaïc** | `core/safety.py`, `core/sensors.py`, `control/chassis_ctrl.py`, `behaviors/avoid_obstacles.py`, `behaviors/navigate_a_to_b.py`, `scenarios/s1_navigation.py` |
| **Mohamed** | `core/camera.py`, `core/logger.py`, `perception/detector.py`, `perception/tracker.py`, `control/pid.py`, `behaviors/follow_person.py`, `scenarios/s2_tracking.py` |
| **Bilal** | `perception/object_locator.py`, `control/arm_ctrl.py`, `behaviors/pick_place.py`, `scenarios/s3_manipulation.py`, `tools/calibration_bras.py` |
| **Pierre-Olivier** | `main.py`, `config.py`, `core/robot_io.py`, `sim/`, `mission/`, `scenarios/s4_mission_complete.py`, `.github/`, README |

Les **tests** de ton module vont dans `tests/unit/` (sans robot) et `tests/robot/` (avec robot).

**Relecteurs** (revue croisée) : Lénaïc relit Mohamed, Mohamed relit Bilal, Bilal relit Lénaïc, et tout le monde relit Pierre-Olivier.

---

## 6. Organisation du code

```
robomaster-projet_EP/
├── main.py            # python main.py s1|s2|s3|mission [--real]
├── config.py          # tous les réglages (seuils, vitesses, bras...)
├── core/              # bas niveau : connexion, capteurs, caméra, sécurité, logs
├── sim/               # faux robot pour travailler sans robot
├── perception/        # YOLO, tracking, localisation d'objet
├── control/           # PID, déplacement, bras
├── behaviors/         # évitement, suivi, navigation, saisie
├── mission/           # machine à états, arbitre de priorités
├── scenarios/         # un fichier par scénario
├── tests/unit/        # tests sans robot
├── tests/robot/       # tests avec robot (au labo)
├── tools/             # outils (calibration, rejeu vidéo)
├── data/              # petits échantillons (vidéos, logs)
├── models/            # poids YOLO (non versionnés)
└── logs/              # journaux (non versionnés)
```

Les dépendances vont **uniquement vers le bas** :
`scenarios → mission → behaviors → perception / control → core`. Un module ne doit jamais importer un module situé plus haut.

Règles de code :
- Un comportement **ne pilote pas le robot** : il retourne une commande `(vx, vy, vz)`.
- Tous les réglages sont dans `config.py`, jamais en dur dans le code.
- Toute commande de vitesse a un timeout, et tout programme finit par un arrêt (`finally`).
- Une seule connexion robot par exécution (`core/robot_io.py`).

---

## 7. Mode simulation et mode robot

| | Simulation (chez soi) | Vrai robot (au labo) |
|---|---|---|
| Lancer | `python main.py s1` | `python main.py s1 --real` |
| Tests | `python -m pytest` | `python -m pytest -m robot` |

Au labo :
1. **Installer tout ce qui demande Internet AVANT** de se connecter au Wi-Fi du robot (pip, modèles YOLO) : le Wi-Fi du robot n'a pas Internet.
2. Se connecter au Wi-Fi `RMEP-xxxxxx`.
3. Un seul PC connecté au robot à la fois (le Dell G15 de référence).
4. Pendant les essais, **enregistrer des vidéos et des logs** dans `data/` : on pourra rejouer chez soi (`python tools/rejouer_video.py data/essai.mp4`).
5. Après la séance : noter les résultats dans le Plan de tests, retirer le label `à-tester-robot` des PR validées, et Pierre-Olivier crée un tag Git (`robot-ok-AAAA-MM-JJ`).

---

## 8. Problèmes fréquents

**« Mon `git push` est refusé sur `main` »**
Normal : `main` est protégé. Crée une branche avec tes commits :
```powershell
git checkout -b ma-branche
git push -u origin ma-branche
```

**« J'ai commité sur `main` par erreur » (avant le push)**
```powershell
git checkout -b ma-branche          # ta branche récupère ton commit
git checkout main
git reset --hard origin/main        # main revient comme sur GitHub
git checkout ma-branche
```
Attention : `reset --hard` efface les modifications non commitées de `main`. Fais-le seulement si tout est déjà dans ta branche.

**« J'ai modifié un fichier par erreur et je veux annuler »**
```powershell
git restore chemin/du/fichier.py    # revient à la dernière version commitée
```

**« J'ai ajouté un mauvais fichier avec `git add` »**
```powershell
git restore --staged chemin/du/fichier.py
```

**« Ma branche est en retard sur `main` / j'ai un conflit »**
```powershell
git checkout main
git pull
git checkout ma-branche
git merge main
```
S'il y a un conflit, Git l'indique et marque le fichier ainsi :
```
<<<<<<< HEAD
ta version
=======
la version des autres
>>>>>>> main
```
Ouvre le fichier dans VS Code : il propose *Accept Current / Incoming / Both*. Garde la bonne version, supprime les lignes `<<<<<<<`, `=======` et `>>>>>>>`, puis :
```powershell
git add le_fichier.py
git commit -m "Résolution du conflit avec main"
python -m pytest
git push
```
Si tu hésites, **demande avant** de valider : on le règle à deux.

**« `pytest` : No module named pytest » ou « No module named robomaster »**
Le venv n'est pas activé : `venv\Scripts\activate` (le `(venv)` doit apparaître).

**« Ça marche chez moi mais pas chez un autre »**
Vérifier `python --version` (3.8.x) et refaire `pip install -r requirements.txt`.

**« Je ne sais plus où j'en suis »**
```powershell
git status          # fichiers modifiés et branche actuelle
git branch          # liste des branches (l'étoile = la branche actuelle)
git log --oneline   # derniers commits
```
**En cas de doute, ne fais rien de destructif : demande dans le groupe.**

---

## 9. Aide-mémoire

```powershell
git checkout main                    # aller sur main
git pull                             # récupérer les nouveautés
git checkout -b s2/ma-tache          # créer ta branche
git status                           # voir l'état
git add fichier.py                   # préparer un fichier
git commit -m "message clair"        # sauvegarder
git push -u origin s2/ma-tache       # envoyer (1ère fois)
git push                             # envoyer (ensuite)
git branch -d s2/ma-tache            # supprimer la branche après merge
python -m pytest                     # tests (sans robot)
python main.py s1                    # lancer en simulation
```
