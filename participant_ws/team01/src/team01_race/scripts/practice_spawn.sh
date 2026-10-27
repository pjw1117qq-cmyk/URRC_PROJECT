#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/jazzy/setup.bash
export GZ_PARTITION="${GZ_PARTITION:-urrc_practice_$(id -u)}"
MODEL="$(ros2 pkg prefix team01_race)/share/team01_race/models/team01_car/model.sdf"
ros2 run ros_gz_sim create --world monza --file "$MODEL" --name team01   --x 0.37867 --y -2.05604 --z 1.24353 -Y 1.45738
