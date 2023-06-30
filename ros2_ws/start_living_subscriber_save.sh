#/bin/bash

cd /ros2_ws
source install/local_setup.bash
ros2 run ros2_network cam_subscriber_save living_subscriber_save living_cam ./living_cam_data 1000 50 50

