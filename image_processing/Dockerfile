# TODO this is unfinished.

# Use an official ROS2 base image
FROM ros:humble

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    ros-humble-cv-bridge \
    ros-humble-geometry-msgs \
    ros-humble-image-msgs \
    ros-humble-kinect-pub \
    ros-humble-std-msgs \
    && rm -rf /var/lib/apt/lists/*

# Set up the ROS2 workspace
WORKDIR /ros_ws

# Copy your ROS2 workspace into the container
COPY . /ros_ws

# Install Python dependencies
RUN pip3 install --no-cache-dir \
    requests \
    transformers==4.27.4 \
    huggingface-hub==0.16.4 \
    Pillow \

    # For lang-segment-anything
    torch \
    torchvision \
    -U git+https://github.com/luca-medeiros/lang-segment-anything.git
    

# Initialize rosdep
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src -r -y

# Build the ROS2 workspace
RUN . /opt/ros/humble/setup.sh && colcon build

# Source the ROS2 setup script
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros_ws/install/setup.bash" >> ~/.bashrc

# Set the entrypoint
ENTRYPOINT ["/ros_ws/install/setup.sh", "&&", "ros2", "run", "<your_package>", "<your_executable>"]

# Default command (can be overridden by `docker run` command)
CMD ["bash"]
