#!/usr/bin/env python3
"""Planar 2R IK/FK for the serial-equivalent EP arm.

Coordinates are metres in arm_base_link: +x forward, +z up. Both joints
rotate about +y, and zero on both puts the arm straight up at full extension —
which is the singular edge of the workspace, so callers should go through
solve() rather than ik().
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

L1 = 0.121
L2 = 0.120

# Shoulder min was -0.274 (~-16 deg), which blocked a real tuck (arm folded
# back). -0.70 (~-40 deg) still clears the chassis and reaches x ≈ -0.20 m.
ARM1_MIN = -0.70
ARM1_MAX = 1.384
ARM2_MIN = -1.8
ARM2_MAX = 0.35

# Annulus the 2R chain can reach before joint limits are considered.
R_MAX = L1 + L2
R_MIN = abs(L1 - L2)
# Keep off the exact boundary: at full extension the Jacobian is singular and
# IK rounding can push cos(elbow) just past 1.
R_EPS = 0.004

GRIPPER_OPEN = 0.001
GRIPPER_CLOSED = -0.023

# Named presets. "joints" entries are commanded directly (sim) so the elbow
# actually folds; Cartesian-only IK for behind-the-shoulder poses leaves arm_2
# near the same angle as extend (~-1.5 rad), which looks unchanged.
# "xyz" entries stay pure Cartesian (x forward, z up in arm_base_link).
PRESETS = {
    # Command joints directly: Cartesian IK for behind-chassis tips leaves the
    # elbow near extend's angle (~-1.5). Pin arm_2 at the URDF limit so the
    # forearm visibly folds.
    "tuck": {"joints": (-0.35, -1.80)},
    "extend": {"xyz": (0.105, 0.142)},
}

# Pose the sim arm is driven to at startup so jogging has room in every
# direction (joint zeros sit on the singularity).
HOME_X = 0.06
HOME_Z = 0.19


def fk(arm_1: float, arm_2: float) -> Tuple[float, float]:
    x = L1 * math.sin(arm_1) + L2 * math.sin(arm_1 + arm_2)
    z = L1 * math.cos(arm_1) + L2 * math.cos(arm_1 + arm_2)
    return x, z


def preset_cartesian(name: str) -> Tuple[float, float]:
    """Tip (x, z) for UI labels / tether moveto."""
    p = PRESETS[name]
    if "joints" in p:
        return fk(*p["joints"])
    return p["xyz"]


def preset_joints(name: str) -> Optional[Tuple[float, float]]:
    p = PRESETS.get(name)
    if p is None:
        return None
    return p.get("joints")


def ik(x: float, z: float) -> Optional[Tuple[float, float]]:
    """Exact solution, or None if unreachable or outside joint limits."""
    r2 = x * x + z * z
    cos_e = (r2 - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    if cos_e < -1.0 or cos_e > 1.0:
        return None
    for arm_2 in (-math.acos(cos_e), math.acos(cos_e)):
        k1 = L1 + L2 * math.cos(arm_2)
        k2 = L2 * math.sin(arm_2)
        arm_1 = math.atan2(x, z) - math.atan2(k2, k1)
        if ARM1_MIN <= arm_1 <= ARM1_MAX and ARM2_MIN <= arm_2 <= ARM2_MAX:
            return arm_1, arm_2
    return None


def nearest_reachable(x: float, z: float) -> Tuple[float, float]:
    """Joint pair inside the limits whose FK is closest to (x, z).

    Coarse grid then two refinement passes. The workspace is small and 2-DoF,
    so this is a few thousand FK evaluations — cheap, and it cannot fail the
    way a closed-form solution can near the limits.
    """
    a1_lo, a1_hi = ARM1_MIN, ARM1_MAX
    a2_lo, a2_hi = ARM2_MIN, ARM2_MAX
    steps = 40
    best = (float("inf"), 0.0, 0.0)
    for _ in range(3):
        for i in range(steps + 1):
            a1 = a1_lo + (a1_hi - a1_lo) * i / steps
            for j in range(steps + 1):
                a2 = a2_lo + (a2_hi - a2_lo) * j / steps
                fx, fz = fk(a1, a2)
                d = (fx - x) ** 2 + (fz - z) ** 2
                if d < best[0]:
                    best = (d, a1, a2)
        _, ba1, ba2 = best
        w1 = (a1_hi - a1_lo) / steps
        w2 = (a2_hi - a2_lo) / steps
        a1_lo, a1_hi = max(ARM1_MIN, ba1 - w1), min(ARM1_MAX, ba1 + w1)
        a2_lo, a2_hi = max(ARM2_MIN, ba2 - w2), min(ARM2_MAX, ba2 + w2)
    return best[1], best[2]


def solve(x: float, z: float) -> Tuple[float, float, List[str]]:
    """Always return a joint pair inside the limits, plus any clamp notes.

    Jogging near the workspace edge is normal use, so an out-of-range goal
    slides to the closest pose the arm can hold instead of being refused.
    """
    notes: List[str] = []
    r = math.hypot(x, z)
    if r < 1e-6:
        x, z, r = 0.0, R_MIN + R_EPS, R_MIN + R_EPS
        notes.append("goal at origin, using min reach")

    r_clamped = min(max(r, R_MIN + R_EPS), R_MAX - R_EPS)
    if abs(r_clamped - r) > 1e-9:
        scale = r_clamped / r
        x, z = x * scale, z * scale
        notes.append(f"reach clamped {r:.3f} -> {r_clamped:.3f} m")

    exact = ik(x, z)
    if exact is not None:
        return exact[0], exact[1], notes

    arm_1, arm_2 = nearest_reachable(x, z)
    fx, fz = fk(arm_1, arm_2)
    notes.append(f"joint limits: nearest pose ({fx:.3f}, {fz:.3f})")
    return arm_1, arm_2, notes


def clamp_joints(arm_1: float, arm_2: float) -> Tuple[float, float]:
    return (
        max(ARM1_MIN, min(ARM1_MAX, arm_1)),
        max(ARM2_MIN, min(ARM2_MAX, arm_2)),
    )
