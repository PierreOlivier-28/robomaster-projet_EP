"""Regulateur PID generique avec zone morte et saturation."""


class PID:
    def __init__(self, kp, ki=0.0, kd=0.0, zone_morte=0.0, sortie_max=None):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.zone_morte = zone_morte
        self.sortie_max = sortie_max
        self._integrale = 0.0
        self._prec = 0.0

    def reset(self):
        self._integrale = 0.0
        self._prec = 0.0

    def step(self, erreur, dt=0.1):
        if abs(erreur) < self.zone_morte:
            erreur = 0.0
        self._integrale += erreur * dt
        derivee = (erreur - self._prec) / dt if dt > 0 else 0.0
        self._prec = erreur
        sortie = self.kp * erreur + self.ki * self._integrale + self.kd * derivee
        if self.sortie_max is not None:
            sortie = max(-self.sortie_max, min(self.sortie_max, sortie))
        return sortie
