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


def test_edf_roll_off_means_zero_body_z_torque():
    """Default vehicle has no EDF roll physics; body-z torque is zero
    regardless of gimbal command."""
    v, env = edf_testbed(), Environment()
    assert v.edf_roll_coeff == 0.0
    s = dyn.initial_state(thrust=v.mass * env.gravity)
    hover = v.mass * env.gravity / v.max_thrust
    cmd = np.array([hover, 0.1, 0.05])  # nonzero gimbals
    d = dyn.state_derivative(s, cmd, v, env)
    # OMEGA derivative z component should be zero.
    assert abs(d[dyn.OMEGA][2]) < 1e-9


def test_edf_roll_on_produces_reaction_torque():
    """With edf_roll_coeff > 0 the body experiences a roll torque
    proportional to thrust, even with zero gimbal."""
    v, env = edf_testbed(), Environment()
    v.edf_roll_coeff = 0.012
    s = dyn.initial_state(thrust=v.mass * env.gravity)
    hover = v.mass * env.gravity / v.max_thrust
    cmd = np.array([hover, 0.0, 0.0])
    d = dyn.state_derivative(s, cmd, v, env)
    # Expected body-z angular acceleration: -roll_coeff * thrust / I_zz.
    expected = -0.012 * v.mass * env.gravity / v.inertia[2, 2]
    assert np.isclose(d[dyn.OMEGA][2], expected, rtol=1e-6)


def test_edf_fan_gyro_couples_yaw_and_pitch():
    """With fan_inertia and fan_omega_max set, a pitch rate couples
    into a yaw torque (gyroscopic precession)."""
    v, env = edf_testbed(), Environment()
    v.edf_fan_inertia = 1e-4
    v.edf_fan_omega_max = 4000.0
    # Start with a pitch rate (about body x) and zero everything else.
    s = dyn.initial_state(thrust=v.mass * env.gravity)
    s[dyn.OMEGA] = np.array([0.5, 0.0, 0.0])  # rad/s about body x
    hover = v.mass * env.gravity / v.max_thrust
    cmd = np.array([hover, 0.0, 0.0])
    d = dyn.state_derivative(s, cmd, v, env)
    # tau_gyro = -omega × H = -(omega_x, 0, 0) × (0, 0, H_z) = (0, omega_x*H_z, 0)
    # So omega_y dot is positive when omega_x is positive and H_z > 0.
    assert d[dyn.OMEGA][1] > 0.0


def test_edf_vane_roll_torque_proportional_to_thrust_and_cmd():
    """With edf_vane_torque_max>0 and a roll_cmd in the 4th cmd slot,
    the body-z torque equals roll_cmd * vane_max * (thrust / Tmax)."""
    v, env = edf_testbed(), Environment()
    v.edf_vane_torque_max = 0.5  # N·m at full thrust
    s = dyn.initial_state(thrust=v.mass * env.gravity)  # hover thrust
    hover = v.mass * env.gravity / v.max_thrust
    # Positive roll command at hover thrust.
    cmd4 = np.array([hover, 0.0, 0.0, 0.6])  # 4-element: include roll_cmd
    d = dyn.state_derivative(s, cmd4, v, env)
    expected_tau = 0.6 * 0.5 * (s[dyn.THRUST] / v.max_thrust)
    expected_alpha = expected_tau / v.inertia[2, 2]
    assert np.isclose(d[dyn.OMEGA][2], expected_alpha, rtol=1e-6)


def test_edf_vane_backward_compatible_with_3element_cmd():
    """When the caller still uses a 3-element command array (no
    roll_cmd), the vane code path stays a no-op even if
    edf_vane_torque_max is set."""
    v, env = edf_testbed(), Environment()
    v.edf_vane_torque_max = 0.5
    s = dyn.initial_state(thrust=v.mass * env.gravity)
    hover = v.mass * env.gravity / v.max_thrust
    cmd3 = np.array([hover, 0.0, 0.0])  # legacy 3-element
    d = dyn.state_derivative(s, cmd3, v, env)
    assert abs(d[dyn.OMEGA][2]) < 1e-9


def test_edf_vane_cancels_edf_roll_reaction_at_hover():
    """The whole point of the vane: when paired with edf_roll_coeff
    and a roll_cmd chosen to exactly counter the reaction torque,
    the net body-z torque cancels and omega_z stays still."""
    v, env = edf_testbed(), Environment()
    v.edf_roll_coeff = 0.012      # reaction torque
    v.edf_vane_torque_max = 0.5    # vane authority
    s = dyn.initial_state(thrust=v.mass * env.gravity)
    hover = v.mass * env.gravity / v.max_thrust
    # Required roll_cmd to cancel:
    #   -roll_coeff * thrust + roll_cmd * vane_max * (thrust/Tmax) = 0
    #   roll_cmd = roll_coeff * Tmax / vane_max
    roll_cmd = v.edf_roll_coeff * v.max_thrust / v.edf_vane_torque_max
    cmd4 = np.array([hover, 0.0, 0.0, roll_cmd])
    d = dyn.state_derivative(s, cmd4, v, env)
    assert abs(d[dyn.OMEGA][2]) < 1e-9


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"ok  {name}")
    print("all tests passed")
