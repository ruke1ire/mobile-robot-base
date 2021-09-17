# mobile-robot-base

This repo is a for a differential drive mobile robot controller that uses 2 motors with encoders and a monocular camera for perception. It uses the ROS2 framework.

Subscribes:
- desired linear and angular velocity

Publishes
- raw camera feed

## Notes:

Publish Camera:
> ros2 run image_tools cam2image --ros-args -p frequency:=2.0

View Published Image:
> ros2 run image_view image_view

