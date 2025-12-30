#!/usr/bin/env bash
set -e  # exit on error

ROS_DISTRO=noetic

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")" && cd ../ && pwd)

# Terminal 1: wild_align
gnome-terminal --tab --title="Wild Align" -- bash -c "
  source /opt/ros/$ROS_DISTRO/setup.bash;
  source $ROOT/src/wild_align/devel/setup.bash;
  roslaunch wild_align run.launch;
  echo 'Launch finished (or interrupted). Press Enter to close...';
    read;
  exec bash"

# Terminal 2: lio_sam
gnome-terminal --tab --title="LIO SAM" -- bash -c "
  source /opt/ros/$ROS_DISTRO/setup.bash;
  source $ROOT/src/slam/devel/setup.bash;
  sleep 1;
  roslaunch lio_sam run.launch;
  echo 'Launch finished (or interrupted). Press Enter to close...';
    read;
  exec bash"

