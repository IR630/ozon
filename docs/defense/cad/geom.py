# -*- coding: utf-8 -*-
"""Geometry of the H4 side-view sketch: descent-chute + reject-tray.

Single source of the drawing's dimensions, shared by the FreeCAD builder
(build_cell_cad.py) and the headless renderer (render_cell_cad.py). Plain data
only — no logic. Units: MILLIMETRES (CAD convention), sim metres ×1000.

Provenance of every number is tagged in PARTS[*]["src"]:
  "sdf"   — measured from sim/worlds/cell_diverter.sdf (real, simulated geometry)
  "calc"  — docs/report/calc_vs_sim.md (belt height 0.40 m)
  "H4"    — design requirement, NOT yet in the SDF (the reject-tray)

Belt top z = 400 (0.40 m, calc_vs_sim). Chute top edge at the belt edge
(y=±250) but z=340 — the item steps 60 mm off the belt onto the chute, then
slides 605 mm at 34.2° to the zone patch at z=0. That 60 mm step is the
"<10 cm descent" that replaces the 0.40 m free fall (physics_oneslide.md
строка 5). The reject-tray at the belt terminus is the only H4 addition.
"""

# Chute incline: sim uses roll 0.597 rad = 34.2°, mu=0.2 (cell_diverter.sdf).
# Design envelope from chute_angle.md is 35–40°; sim sits just under it.
CHUTE_ANGLE_DEG = 34.2

# key annotated dimensions (mm) for the dimensioned drawing
DIMS = {
    "belt_top_z": 400,       # calc_vs_sim: belt height 0.40 m
    "chute_top_z": 340,      # sdf: chute top edge below the belt edge
    "step_descent": 60,      # belt_top_z - chute_top_z: the "<10 cm" step
    "chute_run_y": 500,      # sdf: 250 -> 750 in y
    "chute_len": 605,        # sdf: slope length (1.4x0.605x0.02 plate)
    "belt_width": 500,       # sdf: 0.50 m
    "old_freefall": 400,     # the 0.40 m drop the chute replaces
}

# Each part: box (lx,ly,lz) placed at base (x,y,z), optional roll about X (deg),
# optional mirror across the XZ plane (y -> -y). Colour is RGB 0..1 for render.
PARTS = [
    # belt slab segment (top at z=400, 0.50 m wide, 0.10 m thick)
    {"name": "belt", "box": (800, 500, 100), "base": (-400, -250, 300),
     "src": "calc/sdf", "color": (0.39, 0.45, 0.55)},

    # descent chute C (+Y): top edge (y=250,z=340) -> bottom (y=750,z=0), 34.2°
    {"name": "chute_c", "box": (1400, 605, 20), "base": (-700, 250, 340),
     "roll_deg": -CHUTE_ANGLE_DEG, "src": "sdf", "color": (0.03, 0.57, 0.60)},
    # descent chute D (-Y): mirror of C across the belt centreline
    {"name": "chute_d", "box": (1400, 605, 20), "base": (-700, 250, 340),
     "roll_deg": -CHUTE_ANGLE_DEG, "mirror_y": True, "src": "sdf",
     "color": (0.03, 0.57, 0.60)},

    # parked diverter blades (0.80x0.03x0.30 m), just above each belt edge
    {"name": "blade_c", "box": (800, 30, 300), "base": (-400, 236, 402),
     "src": "sdf", "color": (0.20, 0.25, 0.32)},
    {"name": "blade_d", "box": (800, 30, 300), "base": (-400, -266, 402),
     "src": "sdf", "color": (0.20, 0.25, 0.32)},

    # zone landing patches at z=0 (context)
    {"name": "zone_c", "box": (1400, 800, 20), "base": (-700, 750, -20),
     "src": "sdf", "color": (0.75, 0.83, 0.96)},
    {"name": "zone_d", "box": (1400, 800, 20), "base": (-700, 750, -20),
     "mirror_y": True, "src": "sdf", "color": (0.99, 0.84, 0.67)},

    # reject-tray at the belt terminus (+X) — H4 addition, NOT in the SDF.
    # A shallow catch basin 150 mm below the belt top for routine anomalies
    # (low confidence / not B,C,D) instead of a line-stopping E-stop.
    {"name": "reject_floor", "box": (400, 500, 20), "base": (450, -250, 250),
     "src": "H4", "color": (0.55, 0.36, 0.86)},
    {"name": "reject_wall_end", "box": (20, 500, 150), "base": (830, -250, 250),
     "src": "H4", "color": (0.55, 0.36, 0.86)},
    {"name": "reject_wall_near", "box": (20, 500, 150), "base": (440, -250, 250),
     "src": "H4", "color": (0.55, 0.36, 0.86)},
]
