#!/usr/bin/env bash
# Showcase: open the cell world in Gazebo GUI and spawn all 11 items in a row
# for eyeball inspection (Karpathy principle 1).
# Run inside WSL from the repo root:  bash scripts/view_items.sh
# HEADLESS=1 runs the server only (CI/smoke use).
cd "$(dirname "$0")/.."
export LIBGL_ALWAYS_SOFTWARE=1
export IGN_GAZEBO_RESOURCE_PATH="$PWD/sim/models"

if [ ! -d sim/models/items/bottle ]; then
    python3 scripts/build_item_models.py
fi

if [ "${HEADLESS:-0}" = "1" ]; then
    ign gazebo -s -r -v 1 sim/worlds/cell.sdf > /tmp/gz_view.log 2>&1 &
else
    ign gazebo -r sim/worlds/cell.sdf > /tmp/gz_view.log 2>&1 &
fi
GZ=$!
sleep 10

x=-2.0
for dir in sim/models/items/*/; do
    slug=$(basename "$dir")
    ign service -s /world/cell/create --reqtype ignition.msgs.EntityFactory \
        --reptype ignition.msgs.Boolean --timeout 5000 \
        --req "sdf_filename: \"$PWD/${dir}model.sdf\", name: \"$slug\", pose: {position: {x: $x, y: -2.0, z: 0.05}}" \
        > /dev/null && echo "spawned: $slug at x=$x" || echo "FAILED: $slug"
    x=$(python3 -c "print($x + 1.0)")
done

echo
echo "Все товары стоят в ряд на y=-2 (перед лентой). Закрыть: Ctrl+C или закрыть окно."
wait $GZ
