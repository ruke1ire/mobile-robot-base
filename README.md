# mobile-robot-base

This repo is a for a differential drive mobile robot controller that uses 2 motors with encoders and a monocular camera for perception. It uses the ROS2 framework.

Subscribes:
- desired linear and angular velocity

Publishes
- raw camera feed

## What I did so far

1. downlaod ubuntu 20.04 os for raspberry pi 
2. setup wifi with netplan
3. sudo apt install ubuntu-desktop for gui
4. build ros2
5. download and install raspi-config to enable camera
6. build the ros2_v4l2_camera package for streaming camera
7. build the ros_control packages
8. build the xacro ros package to make the xacro file work

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

Reference:
1. https://medium.com/swlh/raspberry-pi-ros-2-camera-eef8f8b94304[v4l2_camera]:%20Failed%20getting[v4l2_camera]:%20Failed%20getting%20value%20for%20control%2010027009:%20Permission%20denied%20(13);%20returning%200!%20value%20for%20control%2010027009:%20Permission%20denied%20(13);%20returning%200!


