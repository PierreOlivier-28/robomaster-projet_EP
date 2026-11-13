from control.pid import PID


def test_proportionnel():
    assert PID(kp=2.0).step(3.0) == 6.0


def test_saturation_et_zone_morte():
    pid = PID(kp=10.0, zone_morte=1.0, sortie_max=5.0)
    assert pid.step(0.5) == 0.0
    assert pid.step(10.0) == 5.0
