#!/usr/bin/env bash
# Validate the world and all generated item models with the SDF parser.
# Run from the repo root in an environment with Gazebo Fortress (WSL/Docker):
#   bash scripts/check_sdf.sh
set -u
fails=0
ign sdf -k sim/worlds/cell.sdf > /dev/null && echo "OK: sim/worlds/cell.sdf" || { echo "BAD: sim/worlds/cell.sdf"; fails=1; }
for m in sim/models/items/*/model.sdf; do
    if ign sdf -k "$m" > /dev/null; then
        echo "OK: $m"
    else
        echo "BAD: $m"
        fails=1
    fi
done
exit $fails
