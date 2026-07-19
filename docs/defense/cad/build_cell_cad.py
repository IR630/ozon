# -*- coding: utf-8 -*-
"""Build the H4 side-view cell model in FreeCAD and export STEP + STL.

Run headless:  freecadcmd docs/defense/cad/build_cell_cad.py
Reads geometry from geom.py (same dir). Deterministic: one command regenerates
both the real CAD exchange file (STEP) and the mesh (STL).
"""
import os
import sys

import FreeCAD as App  # noqa: F401  (provided by freecadcmd runtime)
import Part

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import geom  # noqa: E402

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")


def _shape(part):
    lx, ly, lz = part["box"]
    s = Part.makeBox(lx, ly, lz)
    s.Placement = App.Placement(
        App.Vector(*part["base"]),
        App.Rotation(App.Vector(1, 0, 0), part.get("roll_deg", 0.0)),
    )
    if part.get("mirror_y"):
        s = s.mirror(App.Vector(0, 0, 0), App.Vector(0, 1, 0))
    return s


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    shapes = [_shape(p) for p in geom.PARTS]
    comp = Part.makeCompound(shapes)

    step_path = os.path.join(OUT_DIR, "cell_sideview.step")
    stl_path = os.path.join(OUT_DIR, "cell_sideview.stl")
    comp.exportStep(step_path)
    comp.exportStl(stl_path)

    print("parts:", len(shapes))
    print("STEP :", step_path, os.path.getsize(step_path), "bytes")
    print("STL  :", stl_path, os.path.getsize(stl_path), "bytes")


main()
