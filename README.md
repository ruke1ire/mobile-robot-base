# mobile-robot-base

This repo is a for a differential drive mobile robot controller that uses 2 motors with encoders and a monocular camera for perception. It uses the ROS2 framework.

Subscribes:
- desired linear and angular velocity

Publishes
- raw camera feed

## Notes:

Publish Camera:
> ros2 run v4l2_camera v4l2_camera_node

View Published Image:
> ros2 run image_view image_view --ros-args --remap image:=image_raw/uncompressed

Convert Image:
> ros2 run image_transport republish compressed in/compressed:=image_raw/compressed raw out:=image_raw/uncompressed

- seems like the ubuntu should be a 64 bit operating system to be able to install the binaries

Run Controller:
> ros2 launch ros2_control_demo_bringup diffbot_system.launch.py start_rviz:=true

> ros2 launch ros2_control_demo_bringup diffbot_system.launch.py start_camera:=true

> ros2 launch ros2_control_demo_bringup diffbot_system.launch.py 

> ros2 topic pub /diffbot_base_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: -0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

Startup:

Systemd is used to startup the program automatically on every reboot. Following are a few useful commands for monitoring/controlling the program.
> sudo systemctl status mobile_robot_base 

> sudo systemctl start mobile_robot_base 

> sudo systemctl stop mobile_robot_base 

> journalctl -fu mobile_robot_base  

Reference:
1. https://medium.com/swlh/raspberry-pi-ros-2-camera-eef8f8b94304[v4l2_camera]:%20Failed%20getting[v4l2_camera]:%20Failed%20getting%20value%20for%20control%2010027009:%20Permission%20denied%20(13);%20returning%200!%20value%20for%20control%2010027009:%20Permission%20denied%20(13);%20returning%200!


