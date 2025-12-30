#!/usr/bin/env bash
set -e  # exit on error

ROS_DISTRO=noetic
source /opt/ros/$ROS_DISTRO/setup.bash

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")" && cd ../ && pwd)

echo "=== Build wild_align_msgs ==="
cd $ROOT/src/wild_msgs
catkin_make
source devel/setup.bash

echo "=== Build slam (LIO-SAM) ==="
cd $ROOT/src/slam
source $ROOT/src/wild_msgs/devel/setup.bash
catkin_make
source devel/setup.bash

echo "=== Build WiLD-Align ==="
cd $ROOT/src/wild_align
source $ROOT/src/wild_msgs/devel/setup.bash
catkin_make
source devel/setup.bash

echo "=== Build Test Coms ==="
cd $ROOT/src/test_coms
source $ROOT/src/wild_msgs/devel/setup.bash
catkin_make
source devel/setup.bash

echo "=== Build complete ==="

