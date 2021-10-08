#!/bin/bash

# source the environment
source install/setup.bash

# launch the ros nodes
ros2 launch ros2_control_demo_bringup diffbot_system.launch.py start_camera:=true
