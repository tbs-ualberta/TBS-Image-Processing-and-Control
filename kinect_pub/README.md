# Introduction

This file will outline how to integrate the Xbox Kinect V2 into ROS on Ubuntu using a driver library called libfreenect2. This was done as a part of the Wheeled Mobile Manipulator (WMM) project at the Telerobotic and Biorobotic Systems Group lab at the University of Alberta. It aims to support the integration of an RGBD camera into the WMM robot to enable autonomous operation capabilities. For more info about the Xbox Kinect V2's
- FOV see [here](https://smeenk.com/kinect-field-of-view-comparison/). For a visualization of the FOV see [here](https://www.smeenk.com/webgl/kinectfovexplorer.html).
- Camera Parameters see [here](https://github.com/shanilfernando/VRInteraction/tree/master/calibration).
Note: these will be slightly different on each camera. The publisher program provides a ROS service for accessing the factory calibration values stored on the camera, however image registration is also calculated in this program and published to a ROS topic, enabling easy conversion between RGB pixel space and Cartesian space (see below for more details).

## Assumptions

The steps in this document were done with the following assumptions:

- Operating system: Ubuntu 20.04.6 LTS
- ROS version: Noetic
- Python version: 3.8.10
- GCC version: 9.4.0
- Camera: Xbox Kinect V2 (Xbox One)

## Dependencies
- [ROS Noetic](https://wiki.ros.org/noetic)
- [OpenCV2](https://opencv.org/)
- [libfreenect2](https://openkinect.github.io/libfreenect2/) (see below for installation instructions)

# Installing libfreenect2 and its dependencies

1. First, install CUDA by following the instructions shown [here](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/).
2. Next, install the CUDA samples. This is important because libfreenect2 utilizes CUDA samples in its operation. It is installed by cloning the CUDA Samples Github repository and building it, following the instructions given [here](https://github.com/nvidia/cuda-samples).
3. libfreenect2 is installed by following the instructions given [here](https://github.com/OpenKinect/libfreenect2).
    - Note: building this repository can be problematic and require extensive troubleshooting. The package uses CUDA to build by default. This can cause issues. Additionally, the package should be installed to be discoverable by the new ROS package that will be written later. To account for these problems, install with the following parameters:
        
        ```bash
        cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/freenect2 -Dfreenect2_DIR=$HOME/freenect2/lib/cmake/freenect2 -DENABLE_CUDA=OFF
        ```
        
4. A program testing correct driver installation can be run using the following command. This should open a visual representation of the Kinect V2 output:

```bash
./bin/Protonect
```

# C++ Publisher Program
The “kinect_publisher_node.cpp” file takes the output of the Xbox Kinect (accessed through libfreenect2), and publishes it to 4 separate ROS topics at a specified rate. These ROS topics correspond with the depth, RGB, and IR data being output by the Kinect. The final ROS topic corresponds with registration data, which maps the RGB image onto the depth image and provides a simple way to transform points from RGB to Cartesian space and vice versa. For more information on the registration function see [here](https://openkinect.github.io/libfreenect2/classlibfreenect2_1_1Registration.html).
