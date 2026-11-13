"""Arbitre : securite > evitement > suivi/navigation > manipulation."""

ARRET = (0.0, 0.0, 0.0)


def choisir(commandes):
    """`commandes` : liste (par ordre de PRIORITE decroissante) de (vx, vy, vz) ou None.
    Retourne la premiere commande non nulle, sinon l'arret."""
    for cmd in commandes:
        if cmd is not None:
            return cmd
    return ARRET
