"""Distances ToF simulees : fonctions f(t_secondes) -> cm."""
import csv


def constante(cm):
    return lambda t: cm


def onde(min_cm=20.0, max_cm=150.0, periode_s=8.0):
    """Le robot 'approche' puis 'recule' d'un obstacle (triangle)."""
    def f(t):
        phase = (t % periode_s) / periode_s
        tri = 1 - abs(2 * phase - 1)          # 0 -> 1 -> 0
        return max_cm - tri * (max_cm - min_cm)
    return f


def rejouer_csv(chemin, colonne="front_cm"):
    """Rejoue un enregistrement fait au labo (colonnes : t, front_cm)."""
    with open(chemin, newline="") as fh:
        lignes = [(float(r["t"]), float(r[colonne])) for r in csv.DictReader(fh)]

    def f(t):
        valeur = lignes[0][1]
        for ti, vi in lignes:
            if ti > t:
                break
            valeur = vi
        return valeur
    return f
