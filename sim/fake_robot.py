"""Faux robot : meme interface que le SDK robomaster (parties utilisees par le projet)."""
import threading
import time

import config
from sim import fake_tof
from sim.fake_camera import FakeCamera


class FakeChassis:
    def __init__(self):
        self.commandes = []          # historique pour les tests

    def drive_speed(self, x=0, y=0, z=0, timeout=None):
        self.commandes.append((x, y, z))


class FakeSensor:
    def __init__(self, fonction_distance=None):
        self.fonction = fonction_distance or fake_tof.onde()
        self._actif = False

    def sub_distance(self, freq=10, callback=None):
        self._actif = True
        t0 = time.time()

        def boucle():
            while self._actif:
                cm = self.fonction(time.time() - t0)
                mm = [3000.0] * 4
                mm[config.PORT_AVANT] = cm * 10.0
                callback(mm)
                time.sleep(1.0 / freq)

        threading.Thread(target=boucle, daemon=True).start()

    def unsub_distance(self):
        self._actif = False


class FakeArm:
    def moveto(self, x=0, y=0):
        print("[sim] bras -> ({}, {})".format(x, y))


class FakeGripper:
    def open(self, power=50):
        print("[sim] pince ouverte")

    def close(self, power=50):
        print("[sim] pince fermee")


class FakeRobot:
    def __init__(self, fonction_distance=None, source_camera=0):
        self.chassis = FakeChassis()
        self.sensor = FakeSensor(fonction_distance)
        self.camera = FakeCamera(source_camera)
        self.robotic_arm = FakeArm()
        self.gripper = FakeGripper()

    def close(self):
        self.sensor.unsub_distance()
        self.camera.stop_video_stream()
