# Introduction

This file describes the implementation of pose assessment into the Wheeled Mobile Manipulator (WMM) project using Google’s MediaPipe API and a generic USB webcam. This will allow the WMM system to analyze the pose of the user, and make decisions based on this analysis. The preceding text will explain how this is implemented, and give setup steps.

## Assumptions

The steps in this document were done with the following assumptions:

- Operating system: Ubuntu 20.04.6 LTS
- ROS version: Noetic
- Python version: 3.8.10
- GCC version: 9.4.0
- Camera: Generic USB Webcam

# usb_cam Setup

To start, a USB webcam is connected to the computer. This will be mounted on the WMM facing the user and its output will be used for pose assessment. This needs to be integrated into ROS. A package called “usb_cam” can be used for this. A link to its documentation can be found here: https://wiki.ros.org/usb_cam#Pixel_formats.2Fencodings_reference

This package can be installed in the terminal by using the following command:

```bash
sudo apt-get install ros-<your-ros-distro>-usb-cam
```

This package requires some camera configuration, which can be achieved by adjusting certain parameters using a launch file. This launch file can then be used to run a node that will publish the camera’s output to a ROS topic. The following launch file serves as an example:

```xml
<launch>
  <node name="usb_cam" type="usb_cam_node" pkg="usb_cam" output="screen">
    <param name="video_device" value="/dev/video0" />
    <param name="image_width" value="640" />
    <param name="image_height" value="480" />
    <param name="pixel_format" value="yuyv" />
    <param name="io_method" value="mmap" />
    <param name="frame_id" value="camera" />
    <param name="camera_name" value="usb_cam" />
    <param name="camera_info_url" value="" />
    <param name="framerate" value="30" />
    <param name="image_format" value="bgr8" />
  </node>
</launch>
```

Depending on the setup, these values can be modified, however for this use case, the above settings are sufficient. After saving this file in the package directory, the “usb_cam” node can be run using the below command:

```bash
roslaunch your-package-name your-launch-file-name.launch
```

Note: your-package-name and your-launch-file-name must be replaced with the appropriate values. For example:

```bash
roslaunch pose_assessment pose_assessment.launch
```

The above described node publishes camera data to 11 different topics, which can be used to read various data from the camera, however the only topic currently utilized in this project is the “usb_cam/image_raw” topic, which publishes a raw RGB datastream.

# MediaPipe Setup and Pose Assessment Program

The MediaPipe API (https://ai.google.dev/edge/mediapipe/solutions/guide) is used in this project to detect the pose of the user. This can be installed using the following command:

```bash
pip install mediapipe opencv-python
```

The "assess_pose_node.py" Python program was written to calculate the angles of the user’s elbow joints and publish them to corresponding ROS topics. Additionally, if desired, the output of the camera with the results of the pose detection algorithm overlayed can be displayed using OpenCV. To display the output, simply ensure the “DISPLAY_OUTPUT” variable is set to “True”. Furthermore, the assessment rate (or how often the camera image is analyzed) can be set by changing the “ASSESSMENT_RATE” variable.
