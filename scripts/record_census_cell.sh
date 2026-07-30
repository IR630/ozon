#!/usr/bin/env bash
# Record ONE census cell as video. Drop-in replacement for run_skeleton.sh, so the
# whole 33-cell census can be filmed without a second copy of the item list, the
# zone list or the seeded orientations — run_matrix.sh stays the only place that
# knows what a census is:
#
#   CENSUS_VIDEO_DIR=runs/census_video SKELETON="bash scripts/record_census_cell.sh" \
#       CELL_TIMEOUT=600 bash scripts/run_matrix.sh 0 3
#
# Why a wrapper instead of SKELETON="bash scripts/record_skeleton_video.sh": that
# script names its output <slug>_<zone>.mp4, which collides across the three
# orientations of one item — the reel would show pose 3 of every item while the
# caption claimed 33 cells. CELL_OI (exported by run_matrix.sh) breaks the tie.
#
# Recording costs ~61 s per cell against ~44 s unfilmed (measured 31.07, bottle
# oi=0, two-head rig): the spectator render drops RTF under llvmpipe. Give the
# census a wider CELL_TIMEOUT than its 400 s default, or a slow cell is recorded
# TIMEOUT and scores as a routing miss the rig never made.
#
# Clips land under runs/ (gitignored) and are mpeg4, as save_video.py writes them;
# scripts/build_census_reel.py re-encodes to H.264 while assembling.
set -e
cd "$(dirname "$0")/.."

SLUG=${1:?usage: record_census_cell.sh <slug> <B|C|D>}
EXPECT=${2:?expected zone B|C|D}
OI=${CELL_OI:?CELL_OI must be set — run this through run_matrix.sh, not by hand}
OUTDIR=${CENSUS_VIDEO_DIR:-runs/census_video}

mkdir -p "$OUTDIR"

# Keep each cell's NODE log beside its clip. run_skeleton.sh defaults NODE_LOG to
# /tmp, and /tmp does not survive here: a WSL distro shuts down when its last
# process exits and wipes it, which already cost this project the seed-0 census
# logs. It is also the file carrying `heads=N`, the only proof that a clip was
# filmed on the SHIPPED two-head rig — the gap that forced a full re-shoot of the
# demo reel on 30.07. One file per cell, or 33 cells would overwrite one log.
export NODE_LOG="$OUTDIR/nodes_${SLUG}_${OI}.log"

# Exit code mirrors the run verdict (record_skeleton_video.sh contract), which is
# what run_matrix.sh scores the cell on — so filming must not swallow a FAIL.
exec bash scripts/record_skeleton_video.sh "$SLUG" "$EXPECT" \
    "$OUTDIR/cell_${SLUG}_${OI}.mp4"
