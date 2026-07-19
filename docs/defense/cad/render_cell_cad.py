# -*- coding: utf-8 -*-
"""Render the H4 cell model headlessly into two PNGs.

Run:  python3 docs/defense/cad/render_cell_cad.py

  cell_sideview_dims.png — dimensioned side elevations (the drawing):
      panel A = Y-Z cross-section at a divert station (belt, both descent
                chutes, blades, zones, the 60 mm step vs the 0.40 m free fall);
      panel B = X-Z longitudinal at the belt terminus (reject-tray, H4).
  cell_sideview_iso.png  — isometric of the exported STEP/STL (proves the model).

Cross-section polygons come from geom.py (same source the STEP was built from);
the iso loads the tessellated STL, so the two views cross-check each other.
"""
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Polygon as MplPolygon  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402
import numpy as np  # noqa: E402
import trimesh  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import geom  # noqa: E402

OUT = os.path.join(HERE, "out")
SRC_LABEL = {"calc/sdf": "лента: calc_vs_sim + SDF", "sdf": "SDF (симуляция)",
             "H4": "reject-лоток: проект H4 (нет в SDF)"}


def _corners_yz(part):
    """4 (y,z) corners of a part's box projected to the Y-Z plane."""
    _, ly, lz = part["box"]
    _, by, bz = part["base"]
    roll = math.radians(part.get("roll_deg", 0.0))
    c, s = math.cos(roll), math.sin(roll)
    pts = []
    for yl in (0, ly):
        for zl in (0, lz):
            y = by + (yl * c - zl * s)
            z = bz + (yl * s + zl * c)
            pts.append((y, z))
    # order as a rectangle (0,0)(ly,0)(ly,lz)(0,lz)
    pts = [pts[0], pts[2], pts[3], pts[1]]
    if part.get("mirror_y"):
        pts = [(-y, z) for (y, z) in pts]
    return pts


def _corners_xz(part):
    """4 (x,z) corners projected to the X-Z plane (for the reject panel)."""
    lx, _, lz = part["box"]
    bx, _, bz = part["base"]
    return [(bx, bz), (bx + lx, bz), (bx + lx, bz + lz), (bx, bz + lz)]


def _dim(ax, p0, p1, text, off=0, color="#334155", fs=11, rot=0):
    ax.annotate("", xy=p1, xytext=p0,
                arrowprops=dict(arrowstyle="<->", color=color, lw=1.3))
    mx, my = (p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2
    ax.text(mx, my + off, text, ha="center", va="center", fontsize=fs,
            color=color, rotation=rot,
            bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.85))


def draw_dims():
    fig, (axA, axB) = plt.subplots(1, 2, figsize=(16, 7),
                                   gridspec_kw={"width_ratios": [1.55, 1]})
    # ---- panel A: Y-Z cross-section ----
    for p in geom.PARTS:
        if p["name"].startswith("reject"):
            continue
        axA.add_patch(MplPolygon(_corners_yz(p), closed=True,
                                 facecolor=p["color"], edgecolor="#1f2937",
                                 lw=1.2, alpha=0.9, zorder=2))
    d = geom.DIMS
    # belt height 0.40 m (left, clear of everything)
    _dim(axA, (-880, 0), (-880, d["belt_top_z"]), "0,40 м\nвысота ленты",
         off=0, fs=10)
    # belt width, between the parked blades above the belt
    _dim(axA, (-250, 445), (250, 445), "0,50 м", off=20, fs=9)
    axA.text(0, 735, "лопасти-дивёртеры (парковка)", ha="center", fontsize=9,
             color="#374151")
    # --- descent comparison at the +Y belt edge: 60 mm step vs 0.40 m fall ---
    # solid red bracket: the 60 mm step the item now takes onto the chute
    _dim(axA, (255, d["chute_top_z"]), (255, d["belt_top_z"]), "", color="#b91c1c")
    # faded dashed: the 0.40 m free fall it replaces
    axA.annotate("", xy=(300, 0), xytext=(300, d["belt_top_z"]),
                 arrowprops=dict(arrowstyle="->", color="#dc2626", lw=1.3,
                                 linestyle="--", alpha=0.6))
    axA.annotate("Съезд: шаг 60 мм (<10 см)\nвместо своб. падения 0,40 м → 2,80 м/с",
                 xy=(280, 360), xytext=(470, 640), fontsize=9.5, color="#b91c1c",
                 ha="center", va="center",
                 bbox=dict(boxstyle="round,pad=0.25", fc="white", ec="#b91c1c",
                           alpha=0.9),
                 arrowprops=dict(arrowstyle="->", color="#b91c1c", lw=1.1))
    # --- chute label block, in the open area above zone C ---
    axA.annotate(f"Склиз {geom.CHUTE_ANGLE_DEG:.1f}° (проект 35–40°)\n"
                 "mu=0,2 · сход 605 мм",
                 xy=(500, 170), xytext=(1180, 560), fontsize=9.5, color="#0e7490",
                 ha="center", va="center",
                 bbox=dict(boxstyle="round,pad=0.25", fc="white", ec="#0e7490",
                           alpha=0.9),
                 arrowprops=dict(arrowstyle="->", color="#0e7490", lw=1.1))
    axA.text(1150, -55, "зона C", fontsize=11, color="#1d4ed8", ha="center")
    axA.text(-1150, -55, "зона D", fontsize=11, color="#c2410c", ha="center")
    axA.set_title("Вид сбоку · поперечное сечение станции дивёрта (Y–Z)",
                  fontsize=12, weight="bold")
    axA.set_xlim(-1650, 1650)
    axA.set_ylim(-90, 760)
    axA.set_aspect("equal")
    axA.set_xlabel("Y, мм (поперёк ленты)")
    axA.set_ylabel("Z, мм (высота)")

    # ---- panel B: X-Z longitudinal at belt terminus (reject-tray) ----
    for p in geom.PARTS:
        if not (p["name"].startswith("reject") or p["name"] == "belt"):
            continue
        col = p["color"] if p["name"].startswith("reject") else (0.39, 0.45, 0.55)
        axB.add_patch(MplPolygon(_corners_xz(p), closed=True, facecolor=col,
                                 edgecolor="#1f2937", lw=1.2, alpha=0.9, zorder=2))
    axB.annotate("", xy=(620, 300), xytext=(360, 400),
                 arrowprops=dict(arrowstyle="->", color="#6d28d9", lw=1.6,
                                 linestyle="--", connectionstyle="arc3,rad=-0.3"))
    axB.text(560, 430, "штатная аномалия\n(низкая уверенность)", fontsize=9,
             color="#6d28d9", ha="center")
    _dim(axB, (900, 250), (900, 400), "150 мм", off=0, color="#6d28d9", fs=9)
    axB.text(640, 150, "Reject-лоток (H4): рутинная аномалия → reject,\n"
                       "аппаратный E-stop — только настоящая авария",
             fontsize=9, color="#4c1d95", ha="center")
    axB.text(0, 440, "лента →", fontsize=10, color="#374151")
    axB.set_title("Вид сбоку · вдоль ленты у схода (X–Z): reject-лоток",
                  fontsize=12, weight="bold")
    axB.set_xlim(-450, 1000)
    axB.set_ylim(100, 620)
    axB.set_aspect("equal")
    axB.set_xlabel("X, мм (вдоль ленты)")
    axB.set_ylabel("Z, мм")

    fig.suptitle("H4 · проектный эскиз ячейки: съезд <10 см и reject-лоток "
                 "(из cell_diverter.sdf + chute_angle.md)", fontsize=13,
                 weight="bold")
    fig.text(0.5, 0.005,
             "Сплошная геометрия — из симуляции (SDF); красный пунктир — "
             "заменяемое свободное падение; фиолет — reject-лоток, проектное "
             "дополнение H4 (в текущем SDF отсутствует).",
             ha="center", fontsize=9, color="#475569")
    fig.tight_layout(rect=[0, 0.03, 1, 0.96])
    path = os.path.join(OUT, "cell_sideview_dims.png")
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return path


def draw_iso():
    mesh = trimesh.load(os.path.join(OUT, "cell_sideview.stl"))
    tris = mesh.triangles  # (n,3,3)
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    coll = Poly3DCollection(tris, facecolor="#7c9cb5", edgecolor="#31465a",
                            linewidths=0.15, alpha=0.9)
    ax.add_collection3d(coll)
    v = mesh.vertices
    for setlim, lo, hi in ((ax.set_xlim, v[:, 0].min(), v[:, 0].max()),
                           (ax.set_ylim, v[:, 1].min(), v[:, 1].max()),
                           (ax.set_zlim, v[:, 2].min(), v[:, 2].max())):
        setlim(lo, hi)
    ax.set_box_aspect((np.ptp(v[:, 0]), np.ptp(v[:, 1]), np.ptp(v[:, 2])))
    ax.view_init(elev=22, azim=-60)
    ax.set_xlabel("X (вдоль ленты)")
    ax.set_ylabel("Y (поперёк)")
    ax.set_zlabel("Z")
    ax.set_title("H4 · изометрия экспортированной модели (cell_sideview.step)",
                 fontsize=12, weight="bold")
    path = os.path.join(OUT, "cell_sideview_iso.png")
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return path


def main():
    os.makedirs(OUT, exist_ok=True)
    print("dims:", draw_dims())
    print("iso :", draw_iso())


if __name__ == "__main__":
    main()
