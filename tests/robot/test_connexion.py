"""Test AVEC robot : pytest -m robot  (PC connecte au Wi-Fi RMEP-xxxxxx)."""
import os

import pytest

pytestmark = pytest.mark.robot


def test_connexion_et_version():
    os.environ["ROBOT_MODE"] = "real"
    import importlib
    import config
    importlib.reload(config)
    from core.robot_io import robot_session
    with robot_session() as ep:
        version = ep.get_version()
        print("Version robot :", version)
        assert version
