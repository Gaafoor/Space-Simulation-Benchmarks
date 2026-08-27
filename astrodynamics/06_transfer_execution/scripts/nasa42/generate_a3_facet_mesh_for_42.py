#!/usr/bin/env python3
"""Generate A3 L1 facet OBJ/MTL for NASA 42 (matches Basilisk spacecraft_geometry L1)."""

from __future__ import annotations

from pathlib import Path

CASE_DIR = Path("/home/mahmedext/Downloads/42 20240919/42/leogeo_benchmark")
MODEL_DIR = Path("/home/mahmedext/Downloads/42 20240919/42/Model")

HX, HY, HZ = 0.50, 0.40, 0.30
PANEL_W, PANEL_H = 0.50, 0.70
# Panels on box ±Y faces (not 0.55 m booms). Basilisk applies facet force at 0.55 m via
# explicit facet centroids; NASA 42 mesh polygons at long lever arms blew up Kane RK4 (~day 33).
PANEL_Y = 0.5 * HY


def _box_vertices() -> list[tuple[float, float, float]]:
    x0, x1 = -0.5 * HX, 0.5 * HX
    y0, y1 = -0.5 * HY, 0.5 * HY
    z0, z1 = -0.5 * HZ, 0.5 * HZ
    return [
        (x0, y0, z0),
        (x1, y0, z0),
        (x1, y1, z0),
        (x0, y1, z0),
        (x0, y0, z1),
        (x1, y0, z1),
        (x1, y1, z1),
        (x0, y1, z1),
    ]


def _panel_vertices(sign: float) -> list[tuple[float, float, float]]:
    y = sign * PANEL_Y
    x0, x1 = -0.5 * PANEL_W, 0.5 * PANEL_W
    z0, z1 = -0.5 * PANEL_H, 0.5 * PANEL_H
    if sign > 0:
        return [(x0, y, z0), (x1, y, z0), (x1, y, z1), (x0, y, z1)]
    return [(x0, y, z1), (x1, y, z1), (x1, y, z0), (x0, y, z0)]


def _emit_face(lines: list[str], i0: int, i1: int, i2: int, i3: int, ni: int) -> None:
    lines.append(f"f {i0}//{ni} {i1}//{ni} {i2}//{ni}")
    lines.append(f"f {i0}//{ni} {i2}//{ni} {i3}//{ni}")


def build_obj() -> str:
    verts = _box_vertices() + _panel_vertices(1.0) + _panel_vertices(-1.0)
    normals = [
        (0.0, 0.0, -1.0),
        (0.0, 0.0, 1.0),
        (0.0, -1.0, 0.0),
        (0.0, 1.0, 0.0),
        (1.0, 0.0, 0.0),
        (-1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, -1.0, 0.0),
    ]
    lines = [
        "# A3 L1 facet mesh — box hub + coplanar solar panels (0.35 m^2 each, Basilisk L1 areas)",
        "mtllib A3_facet_L1.mtl",
        "o A3_hub_panels",
    ]
    for x, y, z in verts:
        lines.append(f"v {x:.8f} {y:.8f} {z:.8f}")
    for nx, ny, nz in normals:
        lines.append(f"vn {nx:.8f} {ny:.8f} {nz:.8f}")

    lines.append("g hub")
    lines.append("usemtl A3_BUS")
    _emit_face(lines, 1, 2, 3, 4, 1)
    _emit_face(lines, 5, 8, 7, 6, 2)
    _emit_face(lines, 1, 5, 6, 2, 3)
    _emit_face(lines, 4, 3, 7, 8, 4)
    _emit_face(lines, 2, 6, 7, 3, 5)
    _emit_face(lines, 1, 4, 8, 5, 6)

    lines.append("g panel_pos_y")
    lines.append("usemtl A3_PANEL")
    _emit_face(lines, 9, 10, 11, 12, 7)

    lines.append("g panel_neg_y")
    lines.append("usemtl A3_PANEL")
    _emit_face(lines, 13, 14, 15, 16, 8)

    return "\n".join(lines) + "\n"


def build_mtl() -> str:
    return """# A3 facet materials — SRP SpecFrac/DiffFrac aligned with Basilisk L1 facets
newmtl A3_BUS
d 1.0
Ns 1.0
Ka 0.2 0.2 0.2
Kd 0.2 0.2 0.2
Ks 0.0 0.0 0.0
Ke 0.0 0.0 0.0
SpecFrac 0.85
DiffFrac 0.15

newmtl A3_PANEL
d 1.0
Ns 50.0
Ka 0.1 0.1 0.15
Kd 0.1 0.1 0.15
Ks 0.2 0.2 0.2
Ke 0.0 0.0 0.0
SpecFrac 0.80
DiffFrac 0.20
"""


def write_mesh(*, targets: tuple[Path, ...] = (CASE_DIR, MODEL_DIR)) -> None:
    obj = build_obj()
    mtl = build_mtl()
    for root in targets:
        root.mkdir(parents=True, exist_ok=True)
        (root / "A3_facet_L1.obj").write_text(obj, encoding="utf-8")
        (root / "A3_facet_L1.mtl").write_text(mtl, encoding="utf-8")


def main() -> int:
    write_mesh()
    print(f"Wrote A3_facet_L1.obj/.mtl to {CASE_DIR} and {MODEL_DIR}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
