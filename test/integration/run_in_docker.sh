#!/usr/bin/env bash
# Runs test/integration/scenarios.py against this checkout inside a ROS 2 Humble container.
# Usage: test/integration/run_in_docker.sh [image]   (default image: ros:humble-ros-base)
set -e
IMAGE="${1:-ros:humble-ros-base}"
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
docker run --rm -v "$REPO:/endpoint:ro" -w /endpoint "$IMAGE" \
    bash -c 'source /opt/ros/*/setup.bash && python3 test/integration/scenarios.py; rc=$?; echo "--- endpoint log ---"; cat /tmp/endpoint_test.log; exit $rc'
