#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
set -u
source "$SCRIPT_DIR/source_practice_env.sh"

if ! ros2 pkg prefix team01_race >/dev/null 2>&1; then
  SEARCH_DIR="$SCRIPT_DIR"
  while [[ "$SEARCH_DIR" != "/" ]]; do
    if [[ -f "$SEARCH_DIR/install/setup.bash" ]]; then
      source "$SEARCH_DIR/install/setup.bash"
      break
    fi
    if [[ -f "$SEARCH_DIR/participant_ws/install/setup.bash" ]]; then
      source "$SEARCH_DIR/participant_ws/install/setup.bash"
      break
    fi
    SEARCH_DIR="$(dirname -- "$SEARCH_DIR")"
  done
fi

if ! ros2 pkg prefix team01_race >/dev/null 2>&1; then
  echo "team01_race is not installed. Build and source the workspace first." >&2
  exit 2
fi

if ! timeout 4 gz topic -l 2>/dev/null | grep -q '^/world/monza/'; then
  echo "Official Monza practice was not found on GZ_PARTITION=$GZ_PARTITION." >&2
  echo "Start official_urrc/run_practice.sh in another terminal first." >&2
  exit 3
fi

exec ros2 launch team01_race practice.launch.py "$@"
