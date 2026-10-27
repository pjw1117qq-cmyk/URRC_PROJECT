#!/usr/bin/env bash
set -euo pipefail
case "${1:-}" in
  go) value=true;; stop) value=false;; *) echo "usage: $0 {go|stop}" >&2; exit 2;; esac
source /opt/ros/jazzy/setup.bash
ros2 topic pub --once /race/start/team01 std_msgs/msg/Bool "{data: $value}"
