# mobile-robot-base

This repo is a for a differential drive mobile robot controller that uses 2 motors with encoders and a monocular camera for perception. It uses the ROS2 framework.

Subscribes:
- desired linear and angular velocity

Publishes
- raw camera feed

## Notes:

Publish Camera:
> ros2 run image_tools cam2image --ros-args -p frequency:=2.0
> ros2 run v4l2_camera v4l2_camera_node

View Published Image:
> ros2 run image_view image_view --ros-args --remap image:=image_raw/uncompressed

Convert Image:
> ros2 run image_transport republish compressed in/compressed:=image_raw/compressed raw out:=image_raw/uncompressed

- The latest repos for image_transport might not work so you can use git checkout to use the older version of the repository which seems to be able to build without error.

Reference:
1. https://medium.com/swlh/raspberry-pi-ros-2-camera-eef8f8b94304[v4l2_camera]:%20Failed%20getting[v4l2_camera]:%20Failed%20getting%20value%20for%20control%2010027009:%20Permission%20denied%20(13);%20returning%200!%20value%20for%20control%2010027009:%20Permission%20denied%20(13);%20returning%200!

