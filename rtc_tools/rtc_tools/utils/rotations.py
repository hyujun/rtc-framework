"""Rotation conventions the tools share.

One copy (design-principles P5): the URDF / catch-frame roll-pitch-yaw was
written out in four tools before this module, and a convention fixed in one
of them would have left the others stale.
"""

from __future__ import annotations

import math

import numpy as np


def rpy_matrix(rpy) -> np.ndarray:
    """URDF fixed-axis roll-pitch-yaw as a rotation matrix:
    ``R = Rz(yaw) · Ry(pitch) · Rx(roll)``, so a vector in the rotated frame
    is ``R @ v`` in the parent frame."""
    r, p, y = (float(v) for v in rpy)
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )
