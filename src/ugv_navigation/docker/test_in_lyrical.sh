#!/usr/bin/env bash
# Build and test ROS packages inside the Lyrical Nav2 image.
#
#   src/ugv_navigation/docker/test_in_lyrical.sh src/ugv_navigation [more package dirs...]
#
# Package dirs are mounted read-only and copied into the container, so nothing
# (build/, install/, log/, __pycache__) is written back to the repo.
# Builds the image first if it does not exist.
set -euo pipefail

IMAGE=ugv-lyrical-nav2
HERE=$(cd "$(dirname "$0")" && pwd)

if [[ $# -eq 0 ]]; then
  echo "usage: $0 <package dir> [package dir...]" >&2
  exit 2
fi

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  docker build -t "$IMAGE" -f "$HERE/lyrical-nav2.Dockerfile" "$HERE"
fi

mounts=()
names=()
for dir in "$@"; do
  abs=$(cd "$dir" && pwd)
  name=$(basename "$abs")
  mounts+=(-v "$abs:/src_ro/$name:ro")
  names+=("$name")
done

docker run --rm "${mounts[@]}" "$IMAGE" bash -c '
  set -e
  mkdir -p /ws/src && cp -r /src_ro/* /ws/src/ && cd /ws
  source /opt/ros/lyrical/setup.bash
  colcon build --packages-select '"${names[*]}"'
  colcon test --packages-select '"${names[*]}"' --event-handlers console_direct+ || true
  colcon test-result --verbose
'
