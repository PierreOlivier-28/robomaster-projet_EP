# Projet DJI RoboMaster EP - ESIGELEC 2026/2027

Robotique mobile et perception : navigation/evitement (S1), tracking de personne (S2),
manipulation d'objet (S3), mission complete.

## Installation (Windows, Python 3.8 OBLIGATOIRE)
Le SDK `robomaster` ne s'installe pas sous Python 3.9+.

```powershell
py -3.8 --version                 # doit afficher 3.8.x
py -3.8 -m venv venv
venv\Scripts\activate
python -m pip install --upgrade pip
pip install -r requirements.txt
```

Faire TOUTES les installations (pip, modeles YOLO) AVANT de se connecter au Wi-Fi du robot :
le mode AP n'a pas d'acces Internet.

## Lancer (mode simulation, sans robot)
```powershell
python main.py s1          # navigation + evitement (faux robot)
python main.py s2
python main.py s3
python main.py mission
pytest                     # tests sans robot
```

## Lancer avec le vrai robot (au labo)
PC connecte au Wi-Fi RMEP-xxxxxx, robot allume :
```powershell
python main.py s1 --real
pytest -m robot            # tests qui necessitent le robot
```

## Organisation
Voir `docs/` et l'onglet "Architecture code" du fichier de gestion de projet.
Dependances vers le bas uniquement :
`scenarios -> mission -> behaviors -> perception / control -> core`.

## Regles d'equipe
- Jamais de push direct sur `main` : branche + pull request + 1 relecture.
- Chacun ne modifie que ses fichiers (voir `.github/CODEOWNERS`).
- `pytest` doit passer avant chaque PR.
- Tout test sur robot est note dans le Plan de tests ; tag Git apres chaque seance robot.
- IA utilisee : l'ecrire dans le Registre IA.
