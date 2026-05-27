"""Quaternion utilities.

Convention: q = [w, x, y, z], unit quaternion representing the rotation that
takes a vector expressed in the BODY frame to the WORLD frame.
"""

from __future__ import annotations

import numpy as np


def quat_normalize(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    if n == 0.0:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / n


def quat_mult(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product a ⊗ b."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ]
    )


def quat_to_rotmat(q: np.ndarray) -> np.ndarray:
    """Rotation matrix R such that v_world = R @ v_body."""
    w, x, y, z = quat_normalize(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def quat_rotate(q: np.ndarray, v_body: np.ndarray) -> np.ndarray:
    """Rotate a body-frame vector into the world frame."""
    return quat_to_rotmat(q) @ v_body


def quat_derivative(q: np.ndarray, omega_body: np.ndarray) -> np.ndarray:
    """dq/dt for body-frame angular velocity omega_body (rad/s)."""
    omega_quat = np.array([0.0, omega_body[0], omega_body[1], omega_body[2]])
    return 0.5 * quat_mult(q, omega_quat)


def quat_to_euler(q: np.ndarray) -> np.ndarray:
    """Return (roll, pitch, yaw) in radians (ZYX / aerospace-ish)."""
    w, x, y, z = quat_normalize(q)
    # roll (x-axis)
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    # pitch (y-axis)
    sinp = 2 * (w * y - z * x)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = np.arcsin(sinp)
    # yaw (z-axis)
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.array([roll, pitch, yaw])


def tilt_angle(q: np.ndarray) -> float:
    """Angle (rad) between the body z-axis and the world z-axis (0 = upright)."""
    body_z_world = quat_rotate(q, np.array([0.0, 0.0, 1.0]))
    cos_t = np.clip(body_z_world[2], -1.0, 1.0)
    return float(np.arccos(cos_t))
