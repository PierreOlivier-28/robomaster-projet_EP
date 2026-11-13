from mission.arbiter import ARRET, choisir


def test_premiere_commande_non_nulle():
    assert choisir([None, (0, 0, 40), (0.3, 0, 0)]) == (0, 0, 40)


def test_arret_si_rien():
    assert choisir([None, None]) == ARRET
