#!/usr/bin/env bash
set -eo pipefail
case "${1:-}" in
  go) value=true;; stop) value=false;; *) echo "usage: $0 {go|stop}" >&2; exit 2;; esac
source /opt/ros/jazzy/setup.bash
set -u
ros2 topic pub /race/start/team01 std_msgs/msg/Bool "{data: $value}" \
  --times 3 \
  --rate 5 \
  --wait-matching-subscriptions 2 \
  --max-wait-time-secs 10 \
  --keep-alive 0.5 \
  --qos-reliability reliable \
  --qos-durability volatile
