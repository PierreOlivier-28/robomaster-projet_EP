"""Faux robot : meme interface que le SDK robomaster (parties utilisees par le projet).

- FakeRobot()            -> console seulement (distances scriptees)
- FakeRobot(world=World) -> relie au simulateur 2D (sim/world2d.py) : les commandes
                            deplacent le robot et les capteurs lisent le monde.
"""
import threading
import time

import config
from sim import fake_tof
from sim.fake_camera import FakeCamera


class FakeChassis:
    def __init__(self, world=None):
        self.world = world
        self.commandes = []          # historique pour les tests
        self._pos_actif = False

    def drive_speed(self, x=0, y=0, z=0, timeout=None):
        self.commandes.append((x, y, z))
        if len(self.commandes) > 5000:
            del self.commandes[:2500]
        if self.world is not None:
            self.world.set_cmd(x, y, z, timeout if timeout else 1.0)

    def sub_position(self, cs=0, freq=5, callback=None):
        """Position relative au depart : (x avant m, y droite m, yaw horaire deg)."""
        self._pos_actif = True

        def boucle():
            while self._pos_actif:
                if self.world is not None:
                    callback(self.world.odometrie())
                else:
                    callback((0.0, 0.0, 0.0))
                time.sleep(1.0 / freq)

        threading.Thread(target=boucle, daemon=True).start()

    def unsub_position(self):
        self._pos_actif = False


class FakeSensor:
    def __init__(self, fonction_distance=None, world=None):
        self.world = world
        self.fonction = fonction_distance or fake_tof.onde()
        self._actif = False

    def sub_distance(self, freq=10, callback=None):
        self._actif = True
        t0 = time.time()

        def boucle():
            while self._actif:
                if self.world is not None:
                    mm = self.world.tof_mm()
                else:
                    cm = self.fonction(time.time() - t0)
                    mm = [3000.0] * 4
                    mm[config.PORT_AVANT] = cm * 10.0
                callback(mm)
                time.sleep(1.0 / freq)

        threading.Thread(target=boucle, daemon=True).start()

    def unsub_distance(self):
        self._actif = False


class _Action:
    """Meme interface que les actions du SDK : arm.moveto(...).wait_for_completed()."""

    def wait_for_completed(self, timeout=None):
        return True


class FakeArm:
    def __init__(self, world=None):
        self.world = world

    def moveto(self, x=0, y=0):
        if self.world is not None:
            self.world.bras_aller(x, y)
        else:
            print("[sim] bras -> ({}, {})".format(x, y))
        return _Action()


class FakeGripper:
    def __init__(self, world=None):
        self.world = world

    def open(self, power=50):
        if self.world is not None:
            self.world.pince_ouvrir()
        else:
            print("[sim] pince ouverte")

    def close(self, power=50):
        if self.world is not None:
            self.world.pince_fermer()
        else:
            print("[sim] pince fermee")

    def objet_saisi(self):
        """Uniquement dans la simulation : la pince tient-elle l'objet ?"""
        return True if self.world is None else self.world.objet_tenu


class FakeRobot:
    def __init__(self, fonction_distance=None, source_camera=0, world=None):
        self.world = world
        self.chassis = FakeChassis(world)
        self.sensor = FakeSensor(fonction_distance, world)
        self.camera = FakeCamera(source_camera)
        self.robotic_arm = FakeArm(world)
        self.gripper = FakeGripper(world)

    def close(self):
        self.sensor.unsub_distance()
        self.chassis.unsub_position()
        self.camera.stop_video_stream()
