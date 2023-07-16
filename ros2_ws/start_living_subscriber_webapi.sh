#/bin/bash

cd /ros2_ws
source install/local_setup.bash
ros2 run ros2_network_py cam_webapi web_api living_cam

