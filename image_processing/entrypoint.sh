#!/bin/bash
source /opt/ros/humble/setup.bash
source /root/ros_ws/install/setup.bash

# Set RMW implementation
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch image_processing object_detection_lite.launch.py


# Keep the container running if the launch fails (for debugging)
while true; do sleep 1000; done