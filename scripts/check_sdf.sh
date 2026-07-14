#!/usr/bin/env bash
# Validate every world and all generated item models with the SDF parser.
# Run from the repo root in an environment with Gazebo Fortress (WSL/Docker):
#   bash scripts/check_sdf.sh
set -u
fails=0
for w in sim/worlds/*.sdf; do
    if ign sdf -k "$w" > /dev/null; then
        echo "OK: $w"
    else
        echo "BAD: $w"
        fails=1
    fi
done
for m in sim/models/items/*/model.sdf; do
    if ign sdf -k "$m" > /dev/null; then
        echo "OK: $m"
    else
        echo "BAD: $m"
        fails=1
    fi
done
exit $fails
