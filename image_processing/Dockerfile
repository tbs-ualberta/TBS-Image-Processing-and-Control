# Use an official ROS2 base image
FROM pytorch/pytorch:latest

# prevent apt from hanging
ARG DEBIAN_FRONTEND=noninteractive

# Get ROS2 key
RUN apt-get update && apt-get install -y software-properties-common curl && \
    add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 packages
RUN apt-get update && apt-get upgrade -y \ 
    && apt-get install ros-humble-ros-base -y

# Source setup script
RUN . /opt/ros/humble/setup.sh

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    ros-humble-cv-bridge \
    # For lang-segment-anything
    git \
    libgl1-mesa-glx \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# Set up the ROS2 workspace
WORKDIR /root/ros_ws/src

# Copy ROS2 workspace into the container
COPY . /root/ros_ws/src/Image-Processing-and-Control/image_processing

# Install lang-segment-anything from source
WORKDIR /root/ros_ws/src/Image-Processing-and-Control/image_processing/lang-segment-anything
RUN pip install -e .

# Install Python dependencies
RUN pip3 install --no-cache-dir \
    requests \
    transformers==4.27.4 \
    huggingface-hub==0.16.4 \
    Pillow

WORKDIR /root/ros_ws

# Initialize rosdep and install dependencies
RUN rosdep init \
    && rosdep fix-permissions \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y

# Build the ROS2 workspace
RUN . /opt/ros/humble/setup.sh && colcon build

# Source the ROS2 setup script
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "source /ros_ws/install/setup.bash" >> ~/.bashrc

RUN ros2 launch image_processing object_detection.launch.py