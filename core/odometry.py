"""Position du robot relative a son point de depart (odometrie des roues).

pose() -> (x avant en m, y droite en m, yaw horaire en deg)
A VERIFIER au labo : signe et unite du 3e champ renvoye par ep.chassis.sub_position().
"""


class Odometrie:
    def __init__(self, ep, freq=10):
        self.ep = ep
        self.freq = freq
        self._pose = (0.0, 0.0, 0.0)

    def _callback(self, info):
        self._pose = (float(info[0]), float(info[1]), float(info[2]))

    def start(self):
        self.ep.chassis.sub_position(cs=0, freq=self.freq, callback=self._callback)

    def stop(self):
        try:
            self.ep.chassis.unsub_position()
        except Exception:
            pass

    def pose(self):
        return self._pose
