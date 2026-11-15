"""Lecture des capteurs ToF (distances en cm, valeurs invalides filtrees)."""
import config


class Distance:
    def __init__(self, ep, freq=10):
        self.ep = ep
        self.freq = freq
        self._mm = []

    def _callback(self, sub_info):
        self._mm = list(sub_info)

    def start(self):
        self.ep.sensor.sub_distance(freq=self.freq, callback=self._callback)

    def stop(self):
        try:
            self.ep.sensor.unsub_distance()
        except Exception:
            pass

    @staticmethod
    def _valide(mm):
        return mm is not None and 0 < mm < 3000

    def cm(self, port):
        """Distance du capteur `port` en cm (config.DISTANCE_INVALIDE_CM si inconnue)."""
        if len(self._mm) <= port or not self._valide(self._mm[port]):
            return config.DISTANCE_INVALIDE_CM
        return self._mm[port] / 10.0

    def front_cm(self):
        return self.cm(config.PORT_AVANT)

    def left_cm(self):
        return self.cm(config.PORT_GAUCHE)

    def right_cm(self):
        return self.cm(config.PORT_DROITE)

    def back_cm(self):
        return self.cm(config.PORT_ARRIERE)

    def min_cm(self):
        """Plus petite distance valide parmi les 4 capteurs."""
        valides = [m / 10.0 for m in self._mm if self._valide(m)]
        return min(valides) if valides else config.DISTANCE_INVALIDE_CM
