#!/usr/bin/env bash
# 2 vs 3 cameras, clean and under noise, through the NEW arbitration combine.
#
# The decision the whole camera thread turns on (docs/decisions.md 2026-07-24): with
# blind cloud-merge gone and role arbitration in (top footprint, side height), does a
# THIRD head buy routing/tolerance a SECOND does not — and does either survive the
# +3 mm sensor noise that a real D435-class line always carries? Runs the full 33-cell
# routing census on four rigs so the clean number and the noisy number sit side by
# side and 2 vs 3 is one table, not an argument.
#
#   bash scripts/census_2v3_noise.sh [seed]        # default seed 0
#
# Each rig writes its own LOGDIR; triage with scripts/triage_matrix.py and the
# organizer tolerance with scripts/census_tolerance.py per dir afterwards. The noisy
# worlds are generated (gitignored) by make_noisy_world.py at sigma 3 mm.
set -e
cd "$(dirname "$0")/.."
SEED=${1:-0}
OUT=runs/c2v3_$(date +%Y%m%d_%H%M%S)_seed${SEED}
mkdir -p "$OUT"

# tag  world                                     bridge
CONFIGS=(
    "2cam_clean sim/worlds/cell_diverter_2cam.sdf        bridge_2cam.yaml"
    "2cam_noisy sim/worlds/cell_diverter_2cam_noisy.sdf  bridge_2cam.yaml"
    "3cam_clean sim/worlds/cell_diverter_3cam.sdf        bridge_3cam.yaml"
    "3cam_noisy sim/worlds/cell_diverter_3cam_noisy.sdf  bridge_3cam.yaml"
)

for cfg in "${CONFIGS[@]}"; do
    read -r tag world bridge <<< "$cfg"
    echo "=== $tag ($world / $bridge) $(date -u +%H:%M:%SZ) ==="
    LOGDIR="$OUT/$tag" WORLD="$world" BRIDGE_CONFIG="$bridge" \
        bash scripts/run_matrix.sh "$SEED" 3 > "$OUT/${tag}.driver.log" 2>&1 || true
    grep -E "routing correctness" "$OUT/$tag/summary.log" 2>/dev/null | tail -1 | sed "s/^/[$tag] /" || echo "[$tag] no summary"
done
echo "=== done -> $OUT ==="
touch "$OUT/DONE"
