# Introduction

This file aims to document the setup of image recognition and depth mapping into the Wheeled Mobile Manipulator (WMM) project. It will outline important steps for setting up the given programs while explaining their functionality.

## Assumptions

The steps in this document were done with the following assumptions:

- Operating system: Ubuntu 20.04.6 LTS
- ROS version: Noetic
- Python version: 3.8.10
- Camera: Xbox Kinect V2 (Xbox One)

# Install LangSAM

First, the “language segment anything” (LangSAM) package needs to be installed. This can be done by following the instructions given here: https://github.com/luca-medeiros/lang-segment-anything.

Note: this project cannot be run in a conda virtual environment; it needs to be run on the main python installation. This is because the dependencies of ROS, LangSAM and it’s dependencies have conflicting versions when installed in a conda environment. Therefore it is best to just install LangSAM into the main python environment using pip.

# Image Display Program

This package contains 5 different python programs. The first of these five programs, “img_display_node.py” simply displays the raw images output by the C++ publisher program. This is helpful for debugging and developing new programs, as it is taken directly from the camera without any processing.

# Image Saving Program

The second program in the package saves the raw images received from the Kinect, as well as the mask images (discussed later) to a specified folder on the computer. The adjustment of various program parameters  is enabled through the manipulation of the constants defined at the top of the file. These include the save directory, folder name, save rate, and a variety of other constants that determine whether each image type is saved or not. For example, if a test is being run where only RGB images need to be saved, the SAVE_RGB constant can be set to true, while all other save selection constants can be set to false. This allows for flexibility during testing and ensures that only important data is saved.

NOTE: if the directory in which files are being saved already exists, this program will clear it before starting the saving process. This enables continued testing without filling up the computer’s disk space. To save the image files of a test permanently, they must either be moved or copied to a separate folder outside of the save folder. Alternatively, the SAVE_FOLDER_NAME and/or SAVE_PATH constants can be changed, leaving the previously saved data intact and creating a new directory to save image data to upon starting the program.

# Image Processing Program

The third program in the package processes the images received from the Kinect and sends the coordinates of the specified target to the WMM control program. It feeds the raw images to an image recognition model along with a prompt. This then calculates a mask over the image for each tracked object, which it then finds the centerpoint of, essentially finding the location of the centroid of each object. It then uses the depth data taken from the camera to calculate the depth value of the centroid of each tracked object. It publishes all this data to a ROS topic using a custom msg structure. The program then compares the calculated masks with the given target phrase, and sends the x (px), y (px), and depth (mm) coordinates of the target centroid to another ROS topic. If two or more targets are detected, the program sends the coordinates of the closest centroid (Note: this can be changed if a different, more complex targeting algorithm is desired).

Note: changing the PROMPT constant will change what the image recognition model will detect. This can be utilized to detect multiple objects in the image. The TARGET constant should contain only one object phrase, and must be contained within the prompt. This allows for detection of multiple objects, the program sends the information of only one target to the robot control algorithm. This could be modified in the future to potentially support obstacle detection.

# Mask Display Program

The final file in the “img_processor” package receives the mask data and displays it using the OpenCV python package. It does this by overlaying each mask in green over the base RGB image. It then draws each centroid over those masks with accompanying text that describes the phrase (or object detected by the program) and the depth of the centroid (in meters). This program also publishes the final image to a ROS topic, which is read by the image saving program for image saving.

# Conclusion

A few notes about using this package:

- Every program in this package relies on the output of the C++ program defined in the kinect_pub package, which reads in data from the Xbox Kinect V2 and publishes it to various ROS topics.
- The image display, saving, and processing programs can be run once the above described C++ program is running, as they only rely on data published by that program.
- The mask display program relies on data published by the processing program, so should only be run after the processing program has started.
