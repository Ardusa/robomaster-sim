#!/usr/bin/env python3
"""Compute exact rigid-body mass properties from a COLLADA mesh.

The URDF in this package was exported from CAD with inertia tensors that do not
correspond to the meshes: several are three to four orders of magnitude too
large, and one failed the triangle inequality outright (no real body can have
those principal moments), which made Gazebo reject the whole model.

Rather than hand-tuning numbers, this recomputes them from the geometry that is
actually simulated. For a uniform-density solid the volume integrals reduce to a
signed sum over tetrahedra formed by each surface triangle and the origin, which
is exact for any closed triangle mesh:

    V   = sum  det(a b c) / 6
    C   = sum  det(a b c) * A C_canonical A^T          (second-moment covariance)
    I   = density * (trace(C_com) * Identity - C_com)

Density is then whatever makes the volume match the mass we declare, so the
result is guaranteed to be a physically realisable tensor.

Usage:
    mesh_inertia.py MESH.dae --mass 0.056 [--offset X Y Z] [--name arm_1_link]
    mesh_inertia.py --batch scripts/inertia_manifest.txt

--offset is the visual/collision <origin> from the URDF: mesh vertices live in
mesh coordinates, and this shifts the reported centre of mass into the link
frame so the emitted <inertial><origin> can be pasted in directly.

Batch mode reads whitespace-separated `link mass mesh [x y z]` records, so the
numbers committed in the xacro files can be regenerated in one command whenever
a mesh or a mass changes. See scripts/inertia_manifest.txt.
"""

from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

_NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}

# Second-moment covariance of the canonical tetrahedron (0, e1, e2, e3) at unit
# density and unit determinant. Any tetrahedron is an affine image of this one.
_C_CANONICAL = np.array([[2.0, 1.0, 1.0], [1.0, 2.0, 1.0], [1.0, 1.0, 2.0]]) / 120.0


def _floats(text: str) -> np.ndarray:
    return np.fromstring(text.replace("\n", " "), sep=" ")


def _source_positions(mesh: ET.Element, vertices_id: str) -> np.ndarray:
    """Resolve a <vertices> reference down to its float_array of xyz triples."""
    source_id = vertices_id
    for vert in mesh.findall("c:vertices", _NS):
        if vert.get("id") == vertices_id.lstrip("#"):
            source_id = vert.find("c:input[@semantic='POSITION']", _NS).get("source")
    source_id = source_id.lstrip("#")
    for src in mesh.findall("c:source", _NS):
        if src.get("id") == source_id:
            return _floats(src.find("c:float_array", _NS).text).reshape(-1, 3)
    raise KeyError(f"no source {source_id!r}")


def _triangles(mesh: ET.Element) -> np.ndarray:
    """Return an (n, 3) array of vertex indices for every surface triangle."""
    faces: list[list[int]] = []
    for prim in list(mesh.findall("c:triangles", _NS)) + list(mesh.findall("c:polylist", _NS)):
        inputs = prim.findall("c:input", _NS)
        stride = max(int(i.get("offset", 0)) for i in inputs) + 1
        vertex_input = prim.find("c:input[@semantic='VERTEX']", _NS)
        offset = int(vertex_input.get("offset", 0))
        indices = _floats(prim.find("c:p", _NS).text).astype(int)[offset::stride]

        vcount = prim.find("c:vcount", _NS)
        if vcount is None:
            counts = [3] * (len(indices) // 3)
        else:
            counts = _floats(vcount.text).astype(int).tolist()

        cursor = 0
        for count in counts:
            polygon = indices[cursor:cursor + count]
            cursor += count
            # Fan-triangulate; these polygons are planar and convex as exported.
            for k in range(1, count - 1):
                faces.append([polygon[0], polygon[k], polygon[k + 1]])
    if not faces:
        raise ValueError("mesh has no triangles")
    return np.array(faces)


def load_mesh(path: Path) -> np.ndarray:
    """Return an (n, 3, 3) array of world-space triangle vertices."""
    root = ET.parse(path).getroot()

    unit = root.find("c:asset/c:unit", _NS)
    scale = float(unit.get("meter", 1.0)) if unit is not None else 1.0
    up_axis = root.findtext("c:asset/c:up_axis", "Y_UP", _NS).strip()

    geometries: dict[str, np.ndarray] = {}
    for geom in root.findall("c:library_geometries/c:geometry", _NS):
        mesh = geom.find("c:mesh", _NS)
        if mesh is None:
            continue
        prim = mesh.find("c:triangles", _NS)
        if prim is None:
            prim = mesh.find("c:polylist", _NS)
        if prim is None:
            continue
        vertex_input = prim.find("c:input[@semantic='VERTEX']", _NS)
        points = _source_positions(mesh, vertex_input.get("source"))
        geometries[geom.get("id")] = points[_triangles(mesh)]

    tris: list[np.ndarray] = []
    for node in root.iter():
        if not node.tag.endswith("}node"):
            continue
        matrix = node.find("c:matrix", _NS)
        transform = _floats(matrix.text).reshape(4, 4) if matrix is not None else np.eye(4)
        for inst in node.findall("c:instance_geometry", _NS):
            local = geometries.get(inst.get("url", "").lstrip("#"))
            if local is None:
                continue
            flat = local.reshape(-1, 3) @ transform[:3, :3].T + transform[:3, 3]
            tris.append(flat.reshape(local.shape))
    if not tris:
        raise ValueError(f"{path.name}: no instantiated geometry")

    out = np.concatenate(tris) * scale
    if up_axis == "Y_UP":
        out = out[..., [0, 2, 1]] * np.array([1.0, -1.0, 1.0])
    elif up_axis == "X_UP":
        out = out[..., [2, 1, 0]] * np.array([1.0, 1.0, -1.0])
    return out


def box_properties(tris: np.ndarray, mass: float) -> tuple[np.ndarray, np.ndarray, float]:
    """Uniform solid box matching the mesh's bounding box.

    Used for open shells, where the volume integrals have no meaning and no
    exact answer exists. It overestimates the moments, which is the safe
    direction: a heavier link resists integration error rather than amplifying it.
    """
    verts = tris.reshape(-1, 3)
    low, high = verts.min(axis=0), verts.max(axis=0)
    size = high - low
    x, y, z = size
    inertia = np.diag([
        mass * (y * y + z * z) / 12.0,
        mass * (x * x + z * z) / 12.0,
        mass * (x * x + y * y) / 12.0,
    ])
    return (low + high) / 2.0, inertia, float(np.prod(size))


def mass_properties(tris: np.ndarray, mass: float) -> tuple[np.ndarray, np.ndarray, float]:
    """Return (centre of mass, inertia about that centre, enclosed volume)."""
    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
    dets = np.einsum("ij,ij->i", a, np.cross(b, c))

    volume = dets.sum() / 6.0
    centroids = (a + b + c) / 4.0
    weighted = (dets[:, None] / 6.0 * centroids).sum(axis=0)

    stacked = np.stack([a, b, c], axis=2)  # columns are the tetra edge vectors
    covariance = np.einsum("n,nij,jk,nlk->il", dets, stacked, _C_CANONICAL, stacked)

    if volume < 0:  # inward winding; the sign cancels consistently
        volume, weighted, covariance = -volume, -weighted, -covariance
    if volume <= 0:
        raise ValueError("degenerate or non-closed mesh: volume is not positive")

    com = weighted / volume
    covariance -= volume * np.outer(com, com)
    density = mass / volume
    inertia = density * (np.trace(covariance) * np.eye(3) - covariance)
    return com, inertia, volume


def is_realisable(inertia: np.ndarray) -> tuple[bool, np.ndarray]:
    """A tensor is realisable only if its principal moments are positive and
    satisfy the triangle inequality; Gazebo rejects models that violate this."""
    moments = np.linalg.eigvalsh(inertia)
    ok = bool((moments > 0).all()) and all(
        moments[i] + moments[j] >= moments[k] - 1e-18
        for i, j, k in ((0, 1, 2), (0, 2, 1), (1, 2, 0))
    )
    return ok, moments


def report(mesh: Path, mass: float, offset, label: str) -> bool:
    tris = load_mesh(mesh)
    bbox_volume = float(np.prod(np.ptp(tris.reshape(-1, 3), axis=0)))

    # Trust the exact integral only when it describes a real solid: a closed
    # surface encloses no more than its bounding box and yields a realisable
    # tensor. A few of these collision meshes are open shells that integrate to
    # a negative principal moment, so this has to be checked, not assumed.
    model = "solid"
    try:
        com, inertia, volume = mass_properties(tris, mass)
        if volume > bbox_volume * 1.001 or not is_realisable(inertia)[0]:
            raise ValueError("integral does not describe a closed solid")
    except ValueError:
        com, inertia, volume = box_properties(tris, mass)
        model = "open shell, bounding box"
    com = com + np.array(offset)
    extent = np.ptp(tris.reshape(-1, 3), axis=0)
    ok, moments = is_realisable(inertia)

    print(f'<!-- {label}: {mesh.name}, {model}, {volume * 1e6:.1f} cm^3, '
          f'rho {mass / volume:.0f} kg/m^3, bbox '
          f'{extent[0]:.3f}x{extent[1]:.3f}x{extent[2]:.3f} m, '
          f'principal {moments[0]:.3g}/{moments[1]:.3g}/{moments[2]:.3g} '
          f'{"valid" if ok else "INVALID"} -->')
    print("<inertial>")
    print(f'  <origin xyz="{com[0]:.7g} {com[1]:.7g} {com[2]:.7g}" rpy="0 0 0" />')
    print(f'  <mass value="{mass:g}" />')
    print(f'  <inertia ixx="{inertia[0, 0]:.6g}" ixy="{inertia[0, 1]:.6g}" '
          f'ixz="{inertia[0, 2]:.6g}" iyy="{inertia[1, 1]:.6g}" '
          f'iyz="{inertia[1, 2]:.6g}" izz="{inertia[2, 2]:.6g}" />')
    print("</inertial>")
    return ok


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mesh", type=Path, nargs="?")
    parser.add_argument("--mass", type=float, help="link mass in kg")
    parser.add_argument("--offset", type=float, nargs=3, default=(0.0, 0.0, 0.0),
                        help="mesh <origin> xyz in the link frame")
    parser.add_argument("--name", default="", help="label for the emitted comment")
    parser.add_argument("--batch", type=Path, help="manifest of link/mass/mesh records")
    args = parser.parse_args()

    if args.batch:
        root = args.batch.resolve().parent.parent
        all_ok = True
        for raw in args.batch.read_text().splitlines():
            line = raw.split("#", 1)[0].split()
            if not line:
                continue
            label, mass, mesh = line[0], float(line[1]), root / line[2]
            offset = [float(v) for v in line[3:6]] if len(line) >= 6 else (0.0, 0.0, 0.0)
            all_ok &= report(mesh, mass, offset, label)
            print()
        return 0 if all_ok else 1

    if args.mesh is None or args.mass is None:
        parser.error("give a MESH and --mass, or use --batch")
    return 0 if report(args.mesh, args.mass, args.offset, args.name or args.mesh.stem) else 1


if __name__ == "__main__":
    raise SystemExit(main())
