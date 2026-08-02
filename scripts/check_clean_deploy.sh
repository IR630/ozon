#!/usr/bin/env bash
# Stage 25 deployment check, one command from the repo root:
#
#   bash scripts/check_clean_deploy.sh                # full gate (needs docker)
#   bash scripts/check_clean_deploy.sh --python-only  # the part that needs no docker
#
# Proves the solution deploys the way the organizers will receive it: a CLEAN
# checkout (git archive HEAD — no gitignored caches, no colcon install/, no
# runs/), the pinned docker image built from scratch sources, and the headless
# CPU end-to-end smoke (scripts/ci_e2e.sh: colcon build + camera -> classifier
# -> diverter episode on the final world) inside that image. This is the same
# path CI's e2e-smoke job takes, packaged for a local/organizer machine.
#
# WHY --python-only EXISTS. A deploy check on Windows 10 LTSC 2019 (02.08) died
# on the docker probe and produced NO signal at all: the operator learned that
# docker was missing and nothing about whether the repository deploys. On that
# OS neither docker nor WSL2 can be installed, so "install docker" is not an
# answer — but the perception/classification core is still fully deployable
# there, and that deployment is worth verifying from a clean checkout rather
# than from someone's working tree. This mode does exactly that and is explicit
# about the half it cannot reach.
#
# The working tree is NOT what is tested — uncommitted changes are invisible
# here by design. Commit first.
set -e
cd "$(dirname "$0")/.."

PYTHON_ONLY=0
[ "${1:-}" = "--python-only" ] && PYTHON_ONLY=1

if [ "$PYTHON_ONLY" -eq 0 ]; then
  # The abort names the way out, not just the missing binary. On LTSC 2019 the
  # answer is a different machine, not an install, and whoever hits this should
  # learn which of the two it is without reading the script.
  command -v docker > /dev/null || {
    echo "ABORT: docker is required for the FULL clean-deploy gate." >&2
    echo "  Он собирает образ из чистого checkout и гоняет в нём сквозной e2e." >&2
    echo "  Windows: нужна 10 версии 21H2 (build 19044)+ или 11." >&2
    echo "  На LTSC 2019 (build 17763) Docker Desktop не ставится в принципе," >&2
    echo "  и WSL2 тоже недоступен — см. docs/deploy-report-2026-08-02.md." >&2
    echo >&2
    echo "  ЧТО МОЖНО СДЕЛАТЬ ЗДЕСЬ И СЕЙЧАС:" >&2
    echo "    bash scripts/check_clean_deploy.sh --python-only" >&2
    echo "  Тот же чистый checkout, свой venv, установка по requirements.txt," >&2
    echo "  линт, тесты и все измерительные скрипты. Исполнительную часть это" >&2
    echo "  не покрывает и об этом честно скажет в конце." >&2
    exit 1
  }
fi

CHECKOUT=$(mktemp -d /tmp/ozon_deploy_check_XXXXXX)
cleanup() {
    docker rm -f ozon-deploy-check-run > /dev/null 2>&1 || true
    rm -rf "$CHECKOUT"
}
trap cleanup EXIT

echo "=== clean checkout of HEAD ($(git rev-parse --short HEAD)) -> $CHECKOUT"
git archive HEAD | tar -x -C "$CHECKOUT"

if [ "$PYTHON_ONLY" -eq 1 ]; then
  PY=${PYTHON:-python3}
  command -v "$PY" > /dev/null || PY=python
  command -v "$PY" > /dev/null || { echo "ABORT: no python3/python on PATH" >&2; exit 1; }

  echo "=== fresh venv from the clean checkout ($($PY --version 2>&1))"
  cd "$CHECKOUT"
  "$PY" -m venv .venv
  # Windows keeps its venv binaries in Scripts/, everything else in bin/.
  VENV_PY="$CHECKOUT/.venv/bin/python"
  [ -x "$VENV_PY" ] || VENV_PY="$CHECKOUT/.venv/Scripts/python.exe"

  echo "=== installing requirements.txt"
  "$VENV_PY" -m pip install --quiet --upgrade pip
  "$VENV_PY" -m pip install --quiet -r requirements.txt

  echo "=== lint"
  "$VENV_PY" -m ruff check .

  echo "=== tests"
  "$VENV_PY" -m pytest -q

  echo "=== item models from STL"
  "$VENV_PY" scripts/build_item_models.py > /dev/null

  echo "=== measurement gates"
  "$VENV_PY" scripts/measure_validation.py
  "$VENV_PY" scripts/measure_private_shapes.py

  echo
  echo "PASS (--python-only): чистый checkout развернулся, линт, тесты и"
  echo "измерительные гейты зелёные."
  echo "НЕ ПОКРЫТО и об этом нельзя делать выводов: ROS 2, Gazebo, colcon,"
  echo "сквозной прогон камера -> классификатор -> дивёртер. Это половина"
  echo "решения, и она проверяется только полным гейтом с docker."
  exit 0
fi

echo "=== building the pinned deployment image"
docker build -t ozon-deploy-check -f "$CHECKOUT/docker/Dockerfile" "$CHECKOUT"

# Same invocation as CI's e2e-smoke job: host network for the ROS graph,
# /dev/shm large enough for Gazebo's transport.
echo "=== headless CPU end-to-end smoke inside the image"
docker run --name ozon-deploy-check-run --network host --shm-size=1g \
    -v "$CHECKOUT:/ws" ozon-deploy-check bash scripts/ci_e2e.sh

echo "PASS: clean checkout built the pinned image and the headless e2e smoke is green"
