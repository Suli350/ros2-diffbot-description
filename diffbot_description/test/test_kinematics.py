import math

import pytest

from diffbot_description.kinematics import (integrate, saturate_wheels, twist_to_wheels,
                                            wheels_to_twist, yaw_to_quaternion)

R, S = 0.05, 0.30


def test_round_trip():
    left, right = twist_to_wheels(0.4, 1.2, R, S)
    v, w = wheels_to_twist(left, right, R, S)
    assert v == pytest.approx(0.4)
    assert w == pytest.approx(1.2)


def test_straight_line_wheels_equal():
    left, right = twist_to_wheels(0.5, 0.0, R, S)
    assert left == pytest.approx(10.0)
    assert right == pytest.approx(10.0)


def test_saturation_keeps_ratio():
    left, right = saturate_wheels(10.0, 40.0, 20.0)
    assert right == pytest.approx(20.0)
    assert left / right == pytest.approx(0.25)


def test_full_circle_returns_home():
    x = y = th = 0.0
    v, w, steps = 0.5, 1.0, 600
    dt = 2 * math.pi / w / steps
    for _ in range(steps):
        x, y, th = integrate(x, y, th, v, w, dt)
    assert x == pytest.approx(0.0, abs=1e-6)
    assert y == pytest.approx(0.0, abs=1e-6)


def test_straight_integration():
    assert integrate(0, 0, math.pi / 2, 1.0, 0.0, 2.0) == pytest.approx((0.0, 2.0, math.pi / 2))


def test_quaternion_is_unit():
    q = yaw_to_quaternion(1.234)
    assert sum(c * c for c in q) == pytest.approx(1.0)
