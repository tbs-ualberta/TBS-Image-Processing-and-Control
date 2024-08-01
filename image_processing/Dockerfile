# Use an official PyTorch base image
FROM pytorch/pytorch:2.0.1-cuda11.7-cudnn8-devel

# prevent apt from hanging
ARG DEBIAN_FRONTEND=noninteractive

# Get ROS2 key and install ROS2 packages and dependencies
RUN apt-get update && apt-get install -y software-properties-common curl && \
    add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && apt-get upgrade -y && \
    apt-get install -y ros-humble-ros-base \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    ros-humble-cv-bridge \
    ros-humble-rosidl-default-generators \
    git \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libstdc++6 && \
    rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install --no-cache-dir \
    requests \
    transformers==4.27.4 \
    huggingface-hub==0.16.4 \
    Pillow \
    lark \
    colcon-common-extensions

# Source ROS setup script
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Clone lang-segment-anything from source
WORKDIR /root
# Official repository (currently broken)
# RUN git clone https://github.com/luca-medeiros/lang-segment-anything
# Fork with fix
RUN git clone https://github.com/MistyDragon7/lang-segment-anything.git 

# Install lang-segment-anything
WORKDIR /root/lang-segment-anything
RUN pip install -e .

# Install image_processing files
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