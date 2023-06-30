#/bin/bash

cd /ros2_ws
source install/local_setup.bash
ros2 run ros2_network cam_publisher living_publisher living_cam 800 600 1000

