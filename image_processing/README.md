# Introduction

This file aims to document the setup of image recognition and depth mapping into the Wheeled Mobile Manipulator (WMM) project. It will outline important steps for setting up the given programs while explaining their functionality.

## Assumptions

The steps in this document were done with the following assumptions:

- Operating system: Ubuntu 22
- ROS version: Humble
- Python version: 3.8.10
- Camera: ZED 2i

# Install LangSAM

First, the “language segment anything” (LangSAM) package needs to be installed. This can be done by following the instructions given [here](https://github.com/luca-medeiros/lang-segment-anything).

**Note:** this project cannot be run in a conda virtual environment; it needs to be run on the main python installation. This is because the dependencies of ROS, LangSAM and it’s dependencies have conflicting versions when installed in a conda environment. Therefore it is best to just install LangSAM into the main python environment using pip.

# Image Display Program

“img_display_node.py” simply displays the raw images output by the camera. This is helpful for debugging and developing new programs, as it is taken directly from the camera without any further processing.

# Image Saving Program

"img_save_node.py" saves the raw images received from the Kinect, as well as the mask images (discussed later) to a specified folder on the computer. The adjustment of various program parameters  is enabled through the manipulation of the constants defined at the top of the file. These include the save directory, folder name, save rate, and a variety of other constants that determine whether each image type is saved or not. For example, if a test is being run where only RGB images need to be saved, the SAVE_RGB constant can be set to true, while all other save selection constants can be set to false. This allows for flexibility during testing and ensures that only important data is saved.

**WARNING:** if the directory in which files are being saved already exists, this program will clear it before starting the saving process. This enables continued testing without filling up the computer’s disk space. To save the image files of a test permanently, they must either be moved or copied to a separate folder outside of the save folder. Alternatively, the SAVE_FOLDER_NAME and/or SAVE_PATH constants can be changed, leaving the previously saved data intact and creating a new directory to save image data to upon starting the program.

# Image Processing Program

"img_processor_node.py" processes the images received from the Kinect and publishes the coordinates of the specified target to a ROS topic. It feeds the raw images to an image recognition model along with a prompt. This then calculates a mask over the image for each tracked object, which it then finds the centerpoint of, essentially finding the location of the centroid of each object. It then uses the depth data taken from the camera to calculate the depth value of the centroid of each tracked object, along with the average depth over the entire object. It publishes all this data to a ROS topic using a custom msg structure. The program then compares the calculated masks with the given target phrase, and sends the x (px), y (px), and depth (mm) coordinates of the target centroid to another ROS topic. If two or more targets are detected, the program sends the coordinates of the closest centroid (Note: this can be changed if a different, more complex targeting algorithm is desired).

**Note:** changing the PROMPT constant will change what the image recognition model will detect. This can be utilized to detect multiple objects in the image. The TARGET constant should contain only one object phrase, and must be contained within the prompt. This allows for detection of multiple objects, the program sends the information of only one target to the robot control algorithm. This could be modified in the future to potentially support obstacle detection.

# Mask Display Program

"mask_display_node" receives the mask data and displays it using the OpenCV python package. It does this by overlaying each mask in green over the base RGB image. It then draws each centroid over those masks with accompanying text that describes the phrase (or object detected by the program) and the depth of the centroid and/or the average depth of the mask (in meters). This program also publishes the final image to a ROS topic, which is read by the image saving program for image saving.

# Obstacle Avoidance Program
"obstacle_avoidance.py" receives the mask data, along with the rgb image and depth data from the image processing program. It then uses the depth data to calculate clusters of similar value depth pixels using the DBSCAN algorithm, which are then labeled as obstacles. The program then calculates the depth (or distance from the camera) of each obstacle and saves it for use in a potential field path planning model. This program is still a work in progress and does not work in its current state.

**Note:** this program assumes that the image processing program's text prompt contains "floor" and that its target is NOT "floor.

# Conclusion

A few notes about using this package:

- The Dockerfiles in this package were written to run on a Nvidia Jetson Orin AGX. This means they build on ARM architecture (not x86). For more details on how to build (including how to build using an x86 machine), see the Docker section below.
- The mask display and obstacle avoidance programs rely on data published by the processing program, so should only be run after the processing program has started.

**Note:** for easiest operation, use the launch files provided in the "launch" directory. Some of these also have parameters which can be changed to change their behaviours. For more information see the launch files.

# For Remote Development using VSCode
1. Install Remote Development extensions:
- Remote - SSH
- Remote - Containers
2. Set up Remote -SSH in VSCode
    1. Open VSCode.
    2. Press Ctrl+Shift+P to open the Command Palette.
    3. Type "Remote-SSH: Connect to Host..." and select it.
    4. Enter the SSH connection string for your Jetson Orin AGX (e.g., your_username@jetson_ip_address).
3. Open a remote folder by clicking the Explorer icon on the sidebar and selecting "Open Folder", then browsing to the desired folder on the Jetson.

# Docker
This project can be built inside of a Docker container using the included Dockerfiles. These Dockerfiles build off an image created by Dustin Franklin and documented in [this](https://github.com/dusty-nv/jetson-containers) repository. There are multiple Dockerfiles to improve modularity and speed up development. It should be possible to simply build Dockerfile.upper immediately, as a working image of Dockerfile.base is available on Docker Hub:
```bash
docker build -f Dockerfile.upper -t image_processing:latest .
```
However, if for whatever reason the Docker Hub image is not working, you can try building locally using Dockerfile.base:
```bash
docker build -f Dockerfile.base -t image_processing_base:latest .
```
The first line in Dockerfile.upper can then be replaced with: ```FROM image_processing_base:latest```

**Note:** To build the Dockerfiles on x86 architecture, [this](https://www.stereolabs.com/docs/docker/building-arm-container-on-x86) guide is helpful.

To run the container and ensure it is connected to the other ROS2 nodes running on the system (either in other containers or natively) run the following:
```bash
docker run -it -v /home/indro/ranger_mini_ws/src/image_processing_and_control:/root/ros_ws/src --network="host" image_processing:latest
```