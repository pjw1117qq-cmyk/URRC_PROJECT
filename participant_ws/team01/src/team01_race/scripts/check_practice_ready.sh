#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
set -u
source "$SCRIPT_DIR/source_practice_env.sh" >/dev/null

failures=0
pass() { printf '%-18s PASS%s\n' "$1" "${2:+ $2}"; }
fail() { printf '%-18s FAIL%s\n' "$1" "${2:+ $2}"; failures=$((failures + 1)); }

gz_topics="$(timeout 5 gz topic -l 2>/dev/null || true)"
if grep -q '^/world/monza/' <<<"$gz_topics"; then
  pass "MONZA WORLD"
else
  fail "MONZA WORLD" "partition=$GZ_PARTITION"
fi

models="$(timeout 5 gz model --list 2>/dev/null || true)"
if grep -qx 'team01' <<<"$models" || grep -Eq '(^|[[:space:]])team01($|[[:space:]])' <<<"$models"; then
  pass "TEAM01 MODEL"
else
  fail "TEAM01 MODEL"
fi

scan_type="$(timeout 5 ros2 topic type /team01/scan 2>/dev/null || true)"
hz_output="$(timeout 7 ros2 topic hz /team01/scan --window 5 2>&1 || true)"
hz="$(sed -n 's/.*average rate: \([0-9.]*\).*/\1/p' <<<"$hz_output" | tail -n 1)"
if [[ "$scan_type" == *"sensor_msgs/msg/LaserScan"* && -n "$hz" ]]; then
  pass "LIDAR" "${hz} Hz"
else
  fail "LIDAR" "topic=/team01/scan"
fi

rear_info="$(timeout 5 ros2 topic info /team01/actuator/rear_left_velocity 2>/dev/null || true)"
if grep -Eq 'Subscription count: [1-9][0-9]*' <<<"$rear_info"; then
  pass "CONTROLLER" "Gazebo joint plugins + bridge"
else
  fail "CONTROLLER" "actuator bridge not connected"
fi

nodes="$(timeout 5 ros2 node list 2>/dev/null || true)"
if grep -qx '/team01_perception' <<<"$nodes" &&
   grep -qx '/team01_decision' <<<"$nodes" &&
   grep -qx '/team01_control_adapter' <<<"$nodes"; then
  pass "AUTONOMY"
else
  fail "AUTONOMY" "race.launch.py is not fully running"
fi

speed_info="$(timeout 5 ros2 topic info /team01/speed 2>/dev/null || true)"
steer_info="$(timeout 5 ros2 topic info /team01/steering 2>/dev/null || true)"
if grep -Eq 'Publisher count: [1-9][0-9]*' <<<"$speed_info" &&
   grep -Eq 'Publisher count: [1-9][0-9]*' <<<"$steer_info"; then
  pass "CONTROL OUTPUTS"
else
  fail "CONTROL OUTPUTS"
fi

start_info="$(timeout 5 ros2 topic info /race/start/team01 2>/dev/null || true)"
if grep -Eq 'Subscription count: [1-9][0-9]*' <<<"$start_info"; then
  pass "START GATE"
else
  fail "START GATE"
fi

echo
if ((failures == 0)); then
  echo "READY FOR PRACTICE"
  exit 0
fi
echo "NOT READY ($failures critical checks failed)"
exit 1
