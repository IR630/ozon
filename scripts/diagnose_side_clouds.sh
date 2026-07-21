#!/usr/bin/env bash
# Side-head clouds, clean rig vs miscalibrated rig, on the SAME resting item.
#
# WHY THIS IS A SCRIPT AND NOT A COMMAND LINE. The census under a +-2 mm / 0.2 deg
# calibration error reads a ~303 mm bottle as 740x505x102 mm on every frame, with
# the y extent pinned at the belt's own 500 mm width — while two exact geometric
# models of that error predict the belt lifts 2.3-4.0 mm against a 5 mm rejection
# margin, i.e. that it must NOT leak. The models and the simulator disagree, so
# the cloud gets looked at rather than modelled again.
#
# Both worlds are dumped in one run because the interesting quantity is the
# DIFFERENCE: the same item, the same pose, the same shipping code, two poses of
# the side heads that differ by the calibration budget and nothing else.
#
#   bash scripts/diagnose_side_clouds.sh [slug]
set -e
cd "$(dirname "$0")/.."

SLUG=${1:-bottle}
# The dumped item rests where dump_item_frame.sh spawns it — under the top head,
# on the belt centre. The crop is sized by the TOP head's dims, so pass what the
# top head actually reported for this item in the census.
ITEM_X=${ITEM_X:-1.5}
ITEM_Y=${ITEM_Y:-0.0}
TOP_DIMS=${TOP_DIMS:-"303 94 91"}

for TAG in clean miscal; do
    if [ "$TAG" = miscal ]; then
        WORLD_FILE=sim/worlds/cell_diverter_3cam_miscal.sdf
        [ -f "$WORLD_FILE" ] || python3 scripts/make_miscal_world.py
    else
        WORLD_FILE=sim/worlds/cell_diverter_3cam.sdf
    fi
    echo "===== dumping $TAG ($WORLD_FILE) ====="
    WORLD="$WORLD_FILE" BRIDGE_CONFIG=sim/bridge_3cam.yaml SIDE=1 FRAMES=1 \
        bash scripts/dump_item_frame.sh "$SLUG" "/tmp/side_diag_$TAG"
done

for TAG in clean miscal; do
    echo
    echo "===== $TAG ====="
    head -4 "/tmp/side_diag_$TAG/pose.txt" 2>/dev/null || true
    python3 scripts/explain_side_cloud.py "/tmp/side_diag_$TAG" \
        --item "$ITEM_X" "$ITEM_Y" 0.4 --dims $TOP_DIMS \
        --png "/tmp/side_cloud_$TAG.png"
done
