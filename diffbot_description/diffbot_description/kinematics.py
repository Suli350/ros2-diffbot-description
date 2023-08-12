"""Differential drive kinematics, kept free of ROS so it is easy to test."""
import math


def twist_to_wheels(v, w, wheel_radius, wheel_separation):
    """Body twist (m/s, rad/s) -> wheel angular velocities (rad/s)."""
    left = (v - w * wheel_separation / 2.0) / wheel_radius
    right = (v + w * wheel_separation / 2.0) / wheel_radius
    return left, right


def wheels_to_twist(left, right, wheel_radius, wheel_separation):
    """Wheel angular velocities (rad/s) -> body twist (m/s, rad/s)."""
    v = wheel_radius * (left + right) / 2.0
    w = wheel_radius * (right - left) / wheel_separation
    return v, w


def saturate_wheels(left, right, max_speed):
    """Scale both wheels equally so neither exceeds max_speed (keeps curvature)."""
    peak = max(abs(left), abs(right))
    if peak <= max_speed or peak == 0.0:
        return left, right
    scale = max_speed / peak
    return left * scale, right * scale


def integrate(x, y, theta, v, w, dt):
    """Exact integration of the unicycle model over dt with constant v, w."""
    if abs(w) < 1e-9:
        return x + v * dt * math.cos(theta), y + v * dt * math.sin(theta), theta
    new_theta = theta + w * dt
    radius = v / w
    x += radius * (math.sin(new_theta) - math.sin(theta))
    y -= radius * (math.cos(new_theta) - math.cos(theta))
    return x, y, math.atan2(math.sin(new_theta), math.cos(new_theta))


def yaw_to_quaternion(yaw):
    """Return (x, y, z, w) for a rotation about Z."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)
