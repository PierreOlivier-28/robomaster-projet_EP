import config
from behaviors import avoid_obstacles


def test_obstacle_proche_tourne():
    cmd = avoid_obstacles.step(config.SEUIL_OBSTACLE_CM - 1)
    assert cmd[0] == 0.0 and cmd[2] != 0.0


def test_voie_libre_ne_fait_rien():
    assert avoid_obstacles.step(config.SEUIL_OBSTACLE_CM + 50) is None
