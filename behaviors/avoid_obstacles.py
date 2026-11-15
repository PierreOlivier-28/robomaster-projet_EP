"""Evitement reactif (Lenaic). Fonction PURE : renvoie une commande, ne pilote pas le robot.

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
