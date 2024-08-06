# Use a PyTorch image built specifically for Jetson from https://github.com/dusty-nv/jetson-containers
FROM torch_ros:r36.3.0-opencv_deb

# Set environment variable to prevent apt from hanging
ARG DEBIAN_FRONTEND=noninteractive

# Install ROS2 dependencies
RUN apt-get update && apt-get install -y \
    software-properties-common \
    curl \
    gnupg \
    lsb-release \
    && add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update \
    && apt-get upgrade -y

# Install ROS2 packages
RUN apt-get install -y \
    ros-humble-ros-base \
    ros-humble-cv-bridge \
    ros-humble-rosidl-default-generators \
    python3-colcon-common-extensions \
    python3-rosdep

# Install other packages
RUN apt-get install -y \
    python3-pip \
    git \
    libgl1-mesa-glx \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install --no-cache-dir \
    requests \
    lark \
    colcon-common-extensions

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
RUN rosdep init && \
    rosdep fix-permissions && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# Build the ROS2 workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --event-handlers console_cohesion+"

# Source the ROS2 setup script
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /root/ros_ws/install/setup.bash" >> ~/.bashrc

# Default command
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /root/ros_ws/install/setup.bash && ros2 launch image_processing object_detection_lite.launch.py"]
