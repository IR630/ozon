#!/usr/bin/env bash
# Physical regression for a same-zone accumulation failure: the first large box
# must clear the C-chute mouth before the following Pouf arrives.
set -euo pipefail
cd "$(dirname "$0")/.."

export SEED=${SEED:-0}
export ORIENT_INDEX=${ORIENT_INDEX:-0}
export RUN_STREAM_POLL_ITERS=${RUN_STREAM_POLL_ITERS:-100}
export LOGDIR=${LOGDIR:-runs/smoke_same_zone_pouf_$(date +%Y%m%d_%H%M%S)}

bash scripts/run_stream.sh box_400x400x300:C:0 pouf:C:1.5
