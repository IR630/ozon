#!/usr/bin/env bash
# Stage 25 deployment check, one command from the repo root (Linux + docker):
#
#   bash scripts/check_clean_deploy.sh
#
# Proves the solution deploys the way the organizers will receive it: a CLEAN
# checkout (git archive HEAD — no gitignored caches, no colcon install/, no
# runs/), the pinned docker image built from scratch sources, and the headless
# CPU end-to-end smoke (scripts/ci_e2e.sh: colcon build + camera -> classifier
# -> diverter episode on the final world) inside that image. This is the same
# path CI's e2e-smoke job takes, packaged for a local/organizer machine.
#
# The working tree is NOT what is tested — uncommitted changes are invisible
# here by design. Commit first.
set -e
cd "$(dirname "$0")/.."

# The abort names the way out, not just the missing binary. A deploy check on a
# Windows LTSC 2019 machine (02.08) died on this line, and "docker is required"
# gave the operator nothing: on that OS Docker Desktop cannot be installed at
# all, so the answer is a different machine, not an install. Whoever hits this
# should learn which of the two it is without reading the script.
command -v docker > /dev/null || {
  echo "ABORT: docker is required for the clean-deploy gate." >&2
  echo "  Это шлюз развёртывания: он собирает образ из чистого checkout." >&2
  echo "  Без docker его пройти нельзя — заглушки тут не будет." >&2
  echo "  Windows: нужна 10 версии 21H2 (build 19044)+ или 11." >&2
  echo "  На LTSC 2019 (build 17763) Docker Desktop не ставится в принципе," >&2
  echo "  и WSL2 тоже недоступен — см. docs/deploy-report-2026-08-02.md." >&2
  echo "  Python-часть при этом работает: pytest -q, ruff check ." >&2
  exit 1
}

CHECKOUT=$(mktemp -d /tmp/ozon_deploy_check_XXXXXX)
cleanup() {
    docker rm -f ozon-deploy-check-run > /dev/null 2>&1 || true
    rm -rf "$CHECKOUT"
}
trap cleanup EXIT

echo "=== clean checkout of HEAD ($(git rev-parse --short HEAD)) -> $CHECKOUT"
git archive HEAD | tar -x -C "$CHECKOUT"

echo "=== building the pinned deployment image"
docker build -t ozon-deploy-check -f "$CHECKOUT/docker/Dockerfile" "$CHECKOUT"

# Same invocation as CI's e2e-smoke job: host network for the ROS graph,
# /dev/shm large enough for Gazebo's transport.
echo "=== headless CPU end-to-end smoke inside the image"
docker run --name ozon-deploy-check-run --network host --shm-size=1g \
    -v "$CHECKOUT:/ws" ozon-deploy-check bash scripts/ci_e2e.sh

echo "PASS: clean checkout built the pinned image and the headless e2e smoke is green"
