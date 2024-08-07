# Use a PyTorch image built specifically for Jetson from https://github.com/dusty-nv/jetson-containers
FROM torch-ros:l4t-r36.3.0

# Set environment variable to prevent apt from hanging
ARG DEBIAN_FRONTEND=noninteractive

# Install ROS2 packages
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-ros-base \
    ros-humble-ament-cmake \
    ros-humble-rclpy \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-rosidl-default-generators \
    ros-humble-std-msgs \
    ros-humble-rosidl-default-runtime \
    ros-humble-ament-lint-auto \
    ros-humble-rmw-cyclonedds-cpp

# Source ROS setup script
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Clone lang-segment-anything from source
WORKDIR /root
RUN git clone https://github.com/luca-medeiros/lang-segment-anything

# Install lang-segment-anything
WORKDIR /root/lang-segment-anything
RUN pip install -e .

# Clone and install image_processing files
WORKDIR /root/ros_ws/src/
RUN git clone https://github.com/tbs-ualberta/WMM-Image-Processing-and-Control.git

WORKDIR /root/ros_ws

# Initialize rosdep and install dependencies
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# Build the ROS2 workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --event-handlers console_cohesion+"

# Source the ROS2 setup script
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /root/ros_ws/install/setup.bash" >> ~/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

# Default command
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /root/ros_ws/install/setup.bash && ros2 launch image_processing object_detection_lite.launch.py"]
