#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""What each head of the three-camera rig actually sees, on the LIVE dumps.

WHY THIS EXISTS. `docs/report/cameras.md` decides the head count on numbers, and
the numbers are argued at length — but the two facts that decide the fork fastest
are visible in one look and were, until this figure, described only in prose:

  * the SIDE heads add HEIGHT (helmet: a dome from above, a silhouette from the
    side), which is the whole of the measured 27/33 -> 30/33 in tolerance;
  * they add NOTHING on the hardest catalogue item. The pen lies 9 mm over the
    belt: from the top it is a sliver, and from either side it does not separate
    from the belt line at all. A rig does not rescue the item that decides the
    C/B border.

A third thing the picture shows was mis-read when this figure was first written,
and the correction matters more than the original claim did. The pale slab on the
right of every `y=-0.90` panel is NOT the opposite head's housing: it is the
roll-cage wall of landing zone C — model at (3.2, 0.9), wall offset (0, 0.6, 0.4),
i.e. world y = 1.5, z 0.005..0.805, which is exactly where the reconstructed
points sit (x 1.98..2.93, y 1.47..1.99, z 0.00..0.80). No camera can appear in any
of these frames at all: the three camera models in `sim/worlds/cell_diverter_3cam.sdf`
carry a `<sensor>` and NO `<visual>` or `<collision>`, and both side clouds hold
exactly ZERO points at the opposite head's |y| in [0.80, 1.00]
(`test_no_head_body_appears_in_any_side_frame`).

The "housing in frame" failure mode of `cameras.md` §4 and
`tests/test_side_head_in_frame.py` therefore remains what that test always
honestly said it was — a GEOMETRIC argument from a datasheet 90 mm housing — and
is not photographed here. It becomes photographable only once the visual props
for the demo video are spawned, which is why that measurement is taken there.

Requires the rig dumps, which are gitignored (`runs/` is a work dir). Refresh with
`wsl -d ozon -- bash runs/g_dump_3cam.sh` (~4 min per item, Gazebo).

    python3 scripts/plot_head_views.py [out.png]
"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.perception import BELT_DEPTH_M, MASK_MARGIN_M, load_depth_png  # noqa: E402

# Three border items, and the reason each is in the figure rather than a catalogue
# sample: helmet is K=0.78 against the 0.8 gate, pen is 9 mm against the 10 mm gate,
# bag is soft (its OBB is the one that tilts). Same three `runs/g_dump_3cam.sh` dumps.
ITEMS = (("helmet", "275x235x235"), ("pen", "148x13x9"), ("bag", "190x190x270"))
HEADS = (("depth_000.png", "TOP z=1.90  (height over belt, mm)"),
         ("depth_side_neg_y_000.png", "SIDE y=-0.90  (range, m)"),
         ("depth_side_pos_y_000.png", "SIDE y=+0.90  (range, m)"))

# Fixed, not per-panel: a shared scale is what makes the rows comparable at a
# glance, and 280 mm covers the tallest item here (bag, 270).
HEIGHT_MAX_MM = 280.0

PANEL_W, PANEL_H = 400, 300
LEFT, TOP, GAP, PAD = 170, 104, 14, 20
FONT = 0  # cv2.FONT_HERSHEY_SIMPLEX; Hershey has no Cyrillic, hence English labels


def _colorize(values, valid, lo, hi):
    """Turbo-mapped uint8 BGR image; pixels outside `valid` come out black.

    Black is not a colour of the scale on purpose: "the sensor returned nothing"
    is a different kind of fact from "the surface is at range X", and the figure
    would lie if the two shared a ramp.
    """
    import cv2

    norm = np.clip((values - lo) / max(hi - lo, 1e-9), 0.0, 1.0)
    img = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    img[~valid] = 0
    return img


def _panel(dump_dir, fname, is_top):
    """One rendered panel and the one number printed on it."""
    import cv2

    depth = load_depth_png(str(dump_dir / fname))
    valid = depth > 0.0
    if is_top:
        height_mm = (BELT_DEPTH_M - depth) * 1000.0
        img = _colorize(height_mm, valid, 0.0, HEIGHT_MAX_MM)
        margin_mm = MASK_MARGIN_M * 1000.0
        note = f"{int((valid & (height_mm > margin_mm)).sum())} px over belt by >{margin_mm:.0f} mm"
    else:
        finite = depth[valid]
        img = _colorize(depth, valid, np.percentile(finite, 1), np.percentile(finite, 99))
        note = f"{int(valid.sum())} px with depth"
    return cv2.resize(img, (PANEL_W, PANEL_H), interpolation=cv2.INTER_AREA), note


def main(argv=None):
    import cv2

    args = list(argv if argv is not None else sys.argv[1:])
    out = Path(args[0]) if args else ROOT / "docs" / "report" / "img" / "head_views_3cam.png"

    w = LEFT + 3 * PANEL_W + 2 * GAP + PAD
    h = TOP + 3 * PANEL_H + 2 * GAP + PAD
    canvas = np.full((h, w, 3), 250, np.uint8)

    for line, txt in enumerate((
            "What each head sees, same item, live Gazebo dumps (runs/frames/*_3cam)",
            "black = no depth returned;  pale slab right of every y=-0.90 panel ="
            " zone C roll cage at y=1.5, NOT a camera (cameras have no visual)",
            "side heads add HEIGHT (helmet) and add nothing to the pen — 9 mm does not"
            " separate from the belt line")):
        cv2.putText(canvas, txt, (PAD, 26 + line * 24), FONT, 0.52, (30, 30, 30), 1,
                    cv2.LINE_AA)

    for col, (_fname, title) in enumerate(HEADS):
        x = LEFT + col * (PANEL_W + GAP)
        cv2.putText(canvas, title, (x, TOP - 10), FONT, 0.5, (30, 30, 30), 1, cv2.LINE_AA)

    for row, (slug, dims) in enumerate(ITEMS):
        dump_dir = ROOT / "runs" / "frames" / f"{slug}_3cam"
        if not dump_dir.is_dir():
            sys.exit(f"нет дампа {dump_dir} — снять: wsl -d ozon -- bash runs/g_dump_3cam.sh")
        y = TOP + row * (PANEL_H + GAP)
        cv2.putText(canvas, slug, (PAD, y + PANEL_H // 2), FONT, 0.6, (30, 30, 30), 1,
                    cv2.LINE_AA)
        cv2.putText(canvas, dims, (PAD, y + PANEL_H // 2 + 22), FONT, 0.42, (110, 110, 110),
                    1, cv2.LINE_AA)
        for col, (fname, _title) in enumerate(HEADS):
            panel, note = _panel(dump_dir, fname, is_top=(col == 0))
            x = LEFT + col * (PANEL_W + GAP)
            canvas[y:y + PANEL_H, x:x + PANEL_W] = panel
            cv2.rectangle(canvas, (x, y), (x + PANEL_W - 1, y + PANEL_H - 1), (170, 170, 170), 1)
            cv2.putText(canvas, note, (x + 8, y + PANEL_H - 10), FONT, 0.42, (255, 255, 255),
                        1, cv2.LINE_AA)
            print(f"{slug:<8} {fname:<26} {note}")

    out.parent.mkdir(parents=True, exist_ok=True)
    ok, buf = cv2.imencode(".png", canvas)      # Unicode-safe write, as elsewhere in repo
    if not ok:
        sys.exit("не удалось закодировать PNG")
    out.write_bytes(buf.tobytes())
    print(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
