"""Sanity checks for the 6-DOF dynamics."""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from rocketsim import dynamics as dyn  # noqa: E402
from rocketsim import quaternion as quat  # noqa: E402
from rocketsim.vehicle import Environment, edf_testbed  # noqa: E402


def test_quaternion_rotation_identity():
    v = np.array([1.0, 2.0, 3.0])
    out = quat.quat_rotate(np.array([1.0, 0, 0, 0]), v)
    assert np.allclose(out, v)


def test_freefall_no_thrust():
    v, env = edf_testbed(), Environment()
    s = dyn.initial_state(position=(0, 0, 100))
    cmd = np.array([0.0, 0.0, 0.0])
    for _ in range(500):  # 1 s at dt=0.002
        s = dyn.rk4_step(s, cmd, 0.002, v, env)
    # z ≈ 100 - 0.5 g t^2 ; vz ≈ -g t
    assert s[dyn.VEL][2] < -9.0
    assert s[dyn.POS][2] < 100.0


def test_hover_thrust_zero_net_accel():
    v, env = edf_testbed(), Environment()
    hover_throttle = v.mass * env.gravity / v.max_thrust
    s = dyn.initial_state(thrust=v.mass * env.gravity)  # already spooled up
    cmd = np.array([hover_throttle, 0.0, 0.0])
    d = dyn.state_derivative(s, cmd, v, env)
    assert np.allclose(d[dyn.VEL], 0.0, atol=1e-6)


def test_gimbal_produces_pitch_torque():
    v, env = edf_testbed(), Environment()
    s = dyn.initial_state(thrust=v.mass * env.gravity)
    # positive gimbal_x should give negative torque about body x (per model)
    cmd = np.array([0.5, np.deg2rad(5.0), 0.0])
    d = dyn.state_derivative(s, cmd, v, env)
    ang_accel = d[dyn.OMEGA]
    assert ang_accel[0] < 0.0
    assert abs(ang_accel[1]) < 1e-9
    assert abs(ang_accel[2]) < 1e-9


def test_quaternion_stays_unit():
    v, env = edf_testbed(), Environment()
    s = dyn.initial_state(omega=(0.5, -0.3, 0.2), thrust=20.0)
    cmd = np.array([0.5, 0.05, -0.03])
    for _ in range(1000):
        s = dyn.rk4_step(s, cmd, 0.002, v, env)
    assert abs(np.linalg.norm(s[dyn.QUAT]) - 1.0) < 1e-6


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"ok  {name}")
    print("all tests passed")
