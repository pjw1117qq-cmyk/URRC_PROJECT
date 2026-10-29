#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
set -u
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/source_practice_env.sh"
MODEL="$(ros2 pkg prefix team01_race)/share/team01_race/models/team01_car/model.sdf"
ros2 run ros_gz_sim create --world monza --file "$MODEL" --name team01 \
  --x 0.37867 --y -2.05604 --z 1.24353 -Y 1.45738
