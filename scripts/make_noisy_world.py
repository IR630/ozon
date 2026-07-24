# -*- coding: utf-8 -*-
"""Bake sensor noise into every head of a world — the one real-line phenomenon
our stand can host itself.

WHY THIS EXISTS. `sim/worlds/*.sdf` carries no `<noise>` element at all (grep,
23.07), so every number this project reports was measured on PERFECT depth. The
offline probe already answered what that costs — the measurement leaves the
organizers' tolerance at sigma = 0.05-0.1 % of range against a D435-class
datasheet of 1-2 %, and what breaks first is SEGMENTATION, not accuracy
(`MASK_MARGIN_M` = 5 mm is thinner than the noise). But an offline probe is not
the contour: it has no belt, no crop, no tracker. The precedent for closing that
gap is `make_miscal_world.py`, which put a calibration error into the WORLD and
found a real defect (`SIDE_BELT_MARGIN_M`) the offline probe could not find in
principle. This script does the same for noise.

WHAT THE MODEL IS AND IS NOT — this matters more than the number it produces:

  * Gazebo's `<noise>` is ADDITIVE GAUSSIAN WITH A CONSTANT stddev in metres.
    A real stereo sensor's error grows with the SQUARE of range, which is why the
    datasheet quotes a percentage. Over our geometry the range spread is narrow
    (head at z=1.9, belt at 0.4, items 0.4-0.7 m tall -> 1.2-1.5 m), so a constant
    stddev is a fair approximation IN THAT BAND and nowhere else.
  * It is uncorrelated per pixel. Real stereo error is spatially correlated and
    comes with whole regions of NO return (black, glossy and transparent bodies).
    Those failure modes stay uncovered — see `docs/report/path_to_line.md`.
  * It is applied to EVERY head the world carries, top and side alike: a rig
    where only the extra heads are noisy would flatter the single-head baseline.

    python3 scripts/make_noisy_world.py                      # 15 mm into the 1-cam world
    python3 scripts/make_noisy_world.py --stddev 0.005
    python3 scripts/make_noisy_world.py --stddev 0.015 \\
        sim/worlds/cell_diverter_3cam_noisy.sdf sim/worlds/cell_diverter_3cam.sdf

FIRST CHECK BEFORE ANY CENSUS ON THE OUTPUT: confirm the noise actually REACHES
the depth channel. Ignition applies `<camera><noise>` to the image pipeline, and
whether an `rgbd_camera`'s depth channel is included is a property of the build,
not of this file. A world that says "noisy" and measures identically to the clean
one is the quiet lie this project exists to avoid, so run one cell on each world
and compare the `item N: WxHxD` lines BEFORE running anything longer:

    WORLD=sim/worlds/cell_diverter.sdf       bash scripts/run_matrix.sh 0 1 0 0
    WORLD=sim/worlds/cell_diverter_noisy.sdf bash scripts/run_matrix.sh 0 1 0 0
"""
import argparse
import re
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parent.parent
SRC_WORLD = _ROOT / "sim" / "worlds" / "cell_diverter.sdf"
OUT_WORLD = _ROOT / "sim" / "worlds" / "cell_diverter_noisy.sdf"

# 1 % of the 1.5 m working distance — the OPTIMISTIC end of the D435-class
# datasheet (1-2 %). Chosen as the default because a world that hides a marginal
# effect is useless: if the contour survives the datasheet, the finer steps below
# are what locate the cliff.
#   0.002 m   already past the 0.75-1.5 mm the offline probe says we hold
#   0.005 m   equal to MASK_MARGIN_M — the segmentation threshold itself
#   0.015 m   D435-class at 1 % of range
DEFAULT_STDDEV_M = 0.015

_NOISE_BLOCK = """  <noise>
              <type>gaussian</type>
              <mean>0</mean>
              <stddev>%s</stddev>
            </noise>
          """

# The <camera> block of every head. Non-greedy so each sensor is matched on its
# own; a greedy match would swallow the whole world and produce one noisy camera.
_CAMERA_RE = re.compile(r"(<camera>.*?)(</camera>)", re.DOTALL)


def cameras_present(sdf_text):
    """Names of the sensors this world carries, in file order."""
    return tuple(re.findall(r'<sensor name="([^"]+)" type="[^"]*camera"', sdf_text))


def add_noise(sdf_text, stddev_m=DEFAULT_STDDEV_M):
    """Return the world with a gaussian noise model in every camera sensor.

    Refuses to touch a world that already declares noise rather than adding a
    second block: two `<noise>` elements in one `<camera>` is undefined behaviour
    that Ignition resolves silently, and a census on such a world would be
    measuring something nobody chose.
    """
    if "<noise>" in sdf_text:
        raise ValueError("world already declares <noise> — refusing to stack a second model")
    out, made = _CAMERA_RE.subn(lambda m: m.group(1) + _NOISE_BLOCK % ("%.9g" % stddev_m)
                                + m.group(2), sdf_text)
    # A `<camera>` block belongs to a sensor here, but SDF also allows one under
    # `<gui>`. Counting the substitutions against the sensors keeps a GUI viewpoint
    # from quietly becoming the thing we made noisy.
    expected = len(cameras_present(sdf_text))
    if made != expected:
        raise ValueError("noised %d <camera> blocks but the world declares %d camera "
                         "sensor(s) — refusing an unattributable world" % (made, expected))
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("out", nargs="?", default=str(OUT_WORLD))
    parser.add_argument("src", nargs="?", default=str(SRC_WORLD))
    parser.add_argument("--stddev", type=float, default=DEFAULT_STDDEV_M,
                        help="depth noise sigma, METRES (default %(default)s)")
    args = parser.parse_args(argv)

    src_path, out_path = Path(args.src), Path(args.out)
    src = src_path.read_text(encoding="utf-8")
    heads = cameras_present(src)
    if not heads:
        print("ABORT: %s carries no camera sensor" % src_path)
        return 2
    out_path.write_text(add_noise(src, args.stddev), encoding="utf-8")
    print("noisy world: %s from %s (gaussian sigma %g m on %d head(s): %s)"
          % (out_path, src_path, args.stddev, len(heads), ", ".join(heads)))
    print("VERIFY BEFORE TRUSTING: run one cell on this world and on the clean one;\n"
          "identical dims mean Ignition did not apply the noise to the depth channel.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
