# Use an official ROS2 base image
FROM ros:humble

# prevent apt from hanging
ARG DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    ros-humble-cv-bridge \
    && rm -rf /var/lib/apt/lists/*

# Set up the ROS2 workspace
WORKDIR /ros_ws/src

# Copy your ROS2 workspace into the container
COPY . /ros_ws/src/image_processing

# For Lang-Segment-Anything
# installing system dependencies:
RUN apt install -y git
RUN apt install libgl1-mesa-glx -y
RUN apt install libglib2.0-0 -y

# Install lang-segment-anything dependencies
RUN cd lang-segment-anything
RUN pip install torch torchvision
RUN pip install -e .

# Install Python dependencies
RUN pip3 install --no-cache-dir \
    requests \
    transformers==4.27.4 \
    huggingface-hub==0.16.4 \
    Pillow

# For image processing
# Initialize rosdep
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src -r -y

# Build the ROS2 workspace
RUN cd /ros_ws
RUN . /opt/ros/humble/setup.sh && colcon build

# Source the ROS2 setup script
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros_ws/install/setup.bash" >> ~/.bashrc

RUN ros2 launch image_processing object_detection.launch.py