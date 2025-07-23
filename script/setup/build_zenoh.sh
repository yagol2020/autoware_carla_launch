#!/bin/bash

source /opt/ros/humble/setup.bash
source $HOME/autoware_carla_launch/env.sh

if [ ! -d "rmw_zenoh_ws" ]; then
    mkdir rmw_zenoh_ws/src -p
    cd rmw_zenoh_ws/src || exit
    git clone https://github.com/ros2/rmw_zenoh.git -b humble
    cd rmw_zenoh || exit
    git checkout 65ded05
    cd ../.. || exit
    sudo apt update # rosdep install need cargo install, so update first
    rosdep update
    rosdep install --from-paths src --ignore-src --rosdistro humble -y
    cd .. || exit
fi

cd rmw_zenoh_ws || exit
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
