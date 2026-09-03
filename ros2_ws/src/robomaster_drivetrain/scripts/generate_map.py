#!/usr/bin/env python3
"""Rasterize axis-aligned collision boxes from a Gazebo SDF world into a Nav2 map."""

from __future__ import annotations

import argparse
import math
import struct
import xml.etree.ElementTree as ET
from pathlib import Path


def _parse_pose(text: str) -> tuple[float, float, float, float, float, float]:
    parts = [float(v) for v in text.split()]
    if len(parts) == 6:
        return tuple(parts)  # type: ignore[return-value]
    raise ValueError(f"bad pose: {text!r}")


def _compose(
    outer: tuple[float, float, float, float, float, float],
    inner: tuple[float, float, float, float, float, float],
) -> tuple[float, float, float, float, float, float]:
    """Compose inner pose expressed in outer frame (yaw-only is enough here)."""
    ox, oy, _, _, _, oyaw = outer
    ix, iy, _, _, _, iyaw = inner
    cos_y = math.cos(oyaw)
    sin_y = math.sin(oyaw)
    wx = ox + cos_y * ix - sin_y * iy
    wy = oy + sin_y * ix + cos_y * iy
    return wx, wy, 0.0, 0.0, 0.0, oyaw + iyaw


def _collect_boxes(
    element: ET.Element,
    parent_pose: tuple[float, float, float, float, float, float],
    boxes: list[tuple[float, float, float, float]],
) -> None:
    pose = parent_pose
    pose_el = element.find("pose")
    if pose_el is not None and pose_el.text:
        pose = _compose(parent_pose, _parse_pose(pose_el.text.strip()))

    for collision in element.findall("collision"):
        cpose = pose
        cp = collision.find("pose")
        if cp is not None and cp.text:
            cpose = _compose(pose, _parse_pose(cp.text.strip()))
        box = collision.find(".//box/size")
        if box is not None and box.text:
            sx, sy, _ = (float(v) for v in box.text.split())
            boxes.append((cpose[0], cpose[1], sx, sy))

    for child in element:
        if child.tag in ("link", "model", "world"):
            _collect_boxes(child, pose, boxes)


def _world_to_cell(
    x: float, y: float, origin_x: float, origin_y: float, res: float, height: int
) -> tuple[int, int]:
    col = int((x - origin_x) / res)
    row = height - 1 - int((y - origin_y) / res)
    return col, row


def _rasterize(
    boxes: list[tuple[float, float, float, float]],
    *,
    origin_x: float,
    origin_y: float,
    width: int,
    height: int,
    res: float,
) -> bytearray:
    grid = bytearray(width * height)
    for cx, cy, sx, sy in boxes:
        half_x = sx / 2.0
        half_y = sy / 2.0
        x0, x1 = cx - half_x, cx + half_x
        y0, y1 = cy - half_y, cy + half_y
        c0, r1 = _world_to_cell(x0, y0, origin_x, origin_y, res, height)
        c1, r0 = _world_to_cell(x1, y1, origin_x, origin_y, res, height)
        c0 = max(0, min(width - 1, c0))
        c1 = max(0, min(width - 1, c1))
        r0 = max(0, min(height - 1, r0))
        r1 = max(0, min(height - 1, r1))
        for row in range(r0, r1 + 1):
            base = row * width
            for col in range(c0, c1 + 1):
                grid[base + col] = 100
    return grid


def _write_pgm(path: Path, grid: bytearray, width: int, height: int) -> None:
    with path.open("wb") as out:
        out.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
        for value in grid:
            out.write(struct.pack("B", 255 - value))


def _write_yaml(path: Path, pgm_name: str, origin_x: float, origin_y: float, res: float) -> None:
    path.write_text(
        "\n".join(
            [
                f"image: {pgm_name}",
                f"resolution: {res}",
                f"origin: [{origin_x}, {origin_y}, 0.0]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.196",
                "",
            ]
        ),
        encoding="utf-8",
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sdf", type=Path)
    parser.add_argument("out_dir", type=Path)
    parser.add_argument("--stem", required=True)
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--origin-x", type=float, default=-7.5)
    parser.add_argument("--origin-y", type=float, default=-8.5)
    parser.add_argument("--width-m", type=float, default=15.0)
    parser.add_argument("--height-m", type=float, default=14.5)
    args = parser.parse_args()

    root = ET.parse(args.sdf).getroot()
    boxes: list[tuple[float, float, float, float]] = []
    _collect_boxes(root, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0), boxes)

    res = args.resolution
    width = int(math.ceil(args.width_m / res))
    height = int(math.ceil(args.height_m / res))
    grid = _rasterize(
        boxes,
        origin_x=args.origin_x,
        origin_y=args.origin_y,
        width=width,
        height=height,
        res=res,
    )

    args.out_dir.mkdir(parents=True, exist_ok=True)
    pgm_path = args.out_dir / f"{args.stem}.pgm"
    yaml_path = args.out_dir / f"{args.stem}.yaml"
    _write_pgm(pgm_path, grid, width, height)
    _write_yaml(yaml_path, pgm_path.name, args.origin_x, args.origin_y, res)
    print(f"wrote {yaml_path} ({width}x{height} @ {res} m/px, {len(boxes)} boxes)")


if __name__ == "__main__":
    main()
