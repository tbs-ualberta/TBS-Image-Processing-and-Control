# Introduction

This document will outline how to integrate the Xbox Kinect V2 into ROS on Ubuntu using a driver library called libfreenect2. This was done as a part of the Wheeled Mobile Manipulator (WMM) project at the Telerobotic and Biorobotic Systems Group lab at the University of Alberta. It aims to support the integration of an RGBD camera into the WMM robot to enable autonomous operation capabilities.

## Assumptions

The steps in this document were done with the following assumptions:

- Operating system: Ubuntu 20.04.6 LTS
- ROS version: Noetic
- Python version: 3.8.10
- GCC version: 9.4.0
- Camera: Xbox Kinect V2 (Xbox One)

# libfreenect2 and Dependency Installation

1. First, install CUDA by following the instructions shown [here](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/).
2. Next, install the CUDA samples. This is important because libfreenect2 utilizes CUDA samples in its operation. It is installed by cloning the following Github repository and building it, following the instructions given in the readme file: https://github.com/nvidia/cuda-samples
3. libfreenect2 is installed by following the instructions given here: https://github.com/OpenKinect/libfreenect2
    - Note: building this repository was problematic and required extensive troubleshooting. The package uses CUDA to build by default. This can cause issues. Additionally, the package should be installed to be discoverable by the new ROS package that will be written later. To account for these problems, install with the following parameters:
        
        ```bash
        cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/freenect2 -Dfreenect2_DIR=$HOME/freenect2/lib/cmake/freenect2 -DENABLE_CUDA=OFF
        ```
        
4. A program testing correct driver installation can be run using the following command. This should open a visual representation of the Kinect V2 output:

```bash
./bin/Protonect
```

# C++ Publisher Program
The “kinect_publisher_node.cpp” file takes the output of the Xbox Kinect (accessed through libfreenect2), and publishes it to 4 separate ROS topics at a specified rate. These ROS topics correspond with the depth, RGB, IR, and combined depth and RGB (defined as reg) data being output by the Kinect.
