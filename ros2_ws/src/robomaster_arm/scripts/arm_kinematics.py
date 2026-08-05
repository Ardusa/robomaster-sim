#!/usr/bin/env python3
"""Planar 2R IK/FK for the serial-equivalent EP arm.

Coordinates are metres in arm_base_link: +x forward, +z up. Both joints
rotate about +y. Link lengths match the URDF origins.
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

L1 = 0.121
L2 = 0.120

ARM1_MIN = -0.274
ARM1_MAX = 1.384
ARM2_MIN = -1.8
ARM2_MAX = 0.35

GRIPPER_OPEN = 0.001
GRIPPER_CLOSED = -0.023


def fk(arm_1: float, arm_2: float) -> Tuple[float, float]:
    x = L1 * math.sin(arm_1) + L2 * math.sin(arm_1 + arm_2)
    z = L1 * math.cos(arm_1) + L2 * math.cos(arm_1 + arm_2)
    return x, z


def ik(x: float, z: float) -> Optional[Tuple[float, float]]:
    r2 = x * x + z * z
    cos_e = (r2 - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    if cos_e < -1.0 or cos_e > 1.0:
        return None
    arm_2 = -math.acos(cos_e)
    k1 = L1 + L2 * math.cos(arm_2)
    k2 = L2 * math.sin(arm_2)
    arm_1 = math.atan2(x, z) - math.atan2(k2, k1)
    if not (ARM1_MIN <= arm_1 <= ARM1_MAX and ARM2_MIN <= arm_2 <= ARM2_MAX):
        arm_2 = math.acos(cos_e)
        k1 = L1 + L2 * math.cos(arm_2)
        k2 = L2 * math.sin(arm_2)
        arm_1 = math.atan2(x, z) - math.atan2(k2, k1)
        if not (ARM1_MIN <= arm_1 <= ARM1_MAX and ARM2_MIN <= arm_2 <= ARM2_MAX):
            return None
    return arm_1, arm_2


def clamp_joints(arm_1: float, arm_2: float) -> Tuple[float, float]:
    return (
        max(ARM1_MIN, min(ARM1_MAX, arm_1)),
        max(ARM2_MIN, min(ARM2_MAX, arm_2)),
    )
