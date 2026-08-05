#!/usr/bin/env python3
"""Fill missing/invalid inertias in AWS RoboMaker house models.

Classic Gazebo tolerated mass-only (or singular) inertial blocks on static
models. Ignition / Gazebo Fortress rejects them and aborts world load with
"A link named link has invalid inertia".

We keep the upstream submodule pristine and patch a copied model tree at
build time. For static scenery any positive-definite diagonal tensor is fine;
use I = m/6 (unit cube of mass m), floored so zeros never sneak through.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

_MASS_RE = re.compile(r"<mass>\s*([0-9.eE+-]+)\s*</mass>")
_INERTIAL_RE = re.compile(r"<inertial\b[^>]*>.*?</inertial>", re.DOTALL)
_INERTIA_RE = re.compile(r"<inertia\b[^>]*>.*?</inertia>", re.DOTALL)
_IXX_RE = re.compile(r"<ixx>\s*([0-9.eE+-]+)\s*</ixx>")
_IYY_RE = re.compile(r"<iyy>\s*([0-9.eE+-]+)\s*</iyy>")
_IZZ_RE = re.compile(r"<izz>\s*([0-9.eE+-]+)\s*</izz>")


def _default_inertia_xml(mass: float, indent: str = "        ") -> str:
    i = max(mass / 6.0, 1e-6)
    return (
        f"{indent}<inertia>\n"
        f"{indent}  <ixx>{i}</ixx>\n"
        f"{indent}  <ixy>0</ixy>\n"
        f"{indent}  <ixz>0</ixz>\n"
        f"{indent}  <iyy>{i}</iyy>\n"
        f"{indent}  <iyz>0</iyz>\n"
        f"{indent}  <izz>{i}</izz>\n"
        f"{indent}</inertia>"
    )


def _diagonal_ok(inertia_xml: str) -> bool:
    vals = []
    for pattern in (_IXX_RE, _IYY_RE, _IZZ_RE):
        match = pattern.search(inertia_xml)
        if match is None:
            return False
        vals.append(float(match.group(1)))
    return all(v > 0.0 for v in vals)


def _patch_inertial(block: str) -> str:
    mass_match = _MASS_RE.search(block)
    mass = float(mass_match.group(1)) if mass_match else 1.0
    inertia_xml = _default_inertia_xml(mass)

    inertia_match = _INERTIA_RE.search(block)
    if inertia_match is None:
        return block.replace("</inertial>", f"{inertia_xml}\n      </inertial>", 1)

    if _diagonal_ok(inertia_match.group(0)):
        return block

    return block[: inertia_match.start()] + inertia_xml + block[inertia_match.end() :]


def patch_sdf(text: str) -> tuple[str, int]:
    patched = 0

    def _replace(match: re.Match[str]) -> str:
        nonlocal patched
        original = match.group(0)
        updated = _patch_inertial(original)
        if updated != original:
            patched += 1
        return updated

    return _INERTIAL_RE.sub(_replace, text), patched


def patch_tree(root: Path) -> int:
    total = 0
    for path in sorted(root.rglob("model.sdf")):
        original = path.read_text(encoding="utf-8")
        updated, count = patch_sdf(original)
        if count:
            path.write_text(updated, encoding="utf-8", newline="\n")
            total += count
            print(f"patched {path.relative_to(root)} ({count} inertial block(s))")
    return total


def verify_tree(root: Path) -> list[str]:
    """Return relative paths that still have invalid inertial blocks."""
    bad: list[str] = []
    for path in sorted(root.rglob("model.sdf")):
        text = path.read_text(encoding="utf-8")
        for block in _INERTIAL_RE.findall(text):
            inertia = _INERTIA_RE.search(block)
            if inertia is None or not _diagonal_ok(inertia.group(0)):
                bad.append(str(path.relative_to(root)))
                break
    return bad


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print(f"usage: {argv[0]} <models_dir>", file=sys.stderr)
        return 2

    root = Path(argv[1])
    if not root.is_dir():
        print(f"not a directory: {root}", file=sys.stderr)
        return 2

    total = patch_tree(root)
    print(f"patched {total} inertial block(s) under {root}")

    bad = verify_tree(root)
    if bad:
        print("still invalid after patch:", file=sys.stderr)
        for path in bad:
            print(f"  {path}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
