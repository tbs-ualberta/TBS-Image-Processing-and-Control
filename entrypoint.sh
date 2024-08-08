#!/bin/bash
source /opt/ros/humble/setup.bash

# Set RMW implementation
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

cd /root/ros_ws

# Update and install dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install --event-handlers console_cohesion+

source /root/ros_ws/install/setup.bash

ros2 launch image_processing object_detection_lite.launch.py

# Keep the container running if the launch fails (for debugging)
while true; do sleep 1000; done