#!/usr/bin/env bash
set -e  # exit on error

ROS_DISTRO=noetic

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")" && cd ../ && pwd)

# Terminal 1: test coms
gnome-terminal --tab --title="Test Coms - Robot 1 <-> Robot 2" -- bash -c "
  source /opt/ros/$ROS_DISTRO/setup.bash;
  source $ROOT/src/test_coms/devel/setup.bash;
  roslaunch coms run.launch;
  echo 'Launch finished (or interrupted). Press Enter to close...';
    read;
  exec bash"

# Terminal 2: lio_sam robot1
gnome-terminal --tab --title="LIO SAM - Robot 1" -- bash -c "
  source /opt/ros/$ROS_DISTRO/setup.bash;
  source $ROOT/src/slam/devel/setup.bash;
  sleep 1;
  roslaunch lio_sam robot1.launch;
  echo 'Launch finished (or interrupted). Press Enter to close...';
    read;
  exec bash"

# Terminal 3: lio_sam robot2
gnome-terminal --tab --title="LIO SAM - Robot 2" -- bash -c "
  source /opt/ros/$ROS_DISTRO/setup.bash;
  source $ROOT/src/slam/devel/setup.bash;
  sleep 2;
  roslaunch lio_sam robot2.launch;
  echo 'Launch finished (or interrupted). Press Enter to close...';
    read;
  exec bash"

# Terminal 4: wild_align robot1
gnome-terminal --tab --title="Wild Align - Robot 1" -- bash -c "
  source /opt/ros/$ROS_DISTRO/setup.bash;
  source $ROOT/src/wild_align/devel/setup.bash;
  sleep 3;
  roslaunch wild_align robot1.launch;
  echo 'Launch finished (or interrupted). Press Enter to close...';
    read;
  exec bash"

# Terminal 5: wild_align robot2
gnome-terminal --tab --title="Wild Align - Robot 2" -- bash -c "
  source /opt/ros/$ROS_DISTRO/setup.bash;
  source $ROOT/src/wild_align/devel/setup.bash;
  sleep 4;
  roslaunch wild_align robot2.launch;
  echo 'Launch finished (or interrupted). Press Enter to close...';
    read;
  exec bash"

