# Introduction

This file documents the image_processing_and_control packages written for the Indro Robotics Ranger Mini 2.0 at the Telerobotic and Biorobotic Systems Group Laboratory at the University of Alberta. The image processing is intended to run in a Docker container, as it contains various delicate dependencies that need are difficult to install properly locally.

## Assumptions

The steps in this document were done with the following assumptions:

- Operating system: Ubuntu 22
- ROS version: Humble
- Python version: 3.8.10
- Camera: ZED 2i

# Install and Run Locally (Not Recommended)
For installation steps, see both the Dockerfile.base and Dockerfile.upper Dockerfiles. These outline important installation steps to get this package running. Importantly, compatible versions of pytorch and ROS2 have to be installed for everything to work correctly, which is sometimes difficult to do. Therefore it is recommended to simply run the Docker image provided.

**Note:** this project cannot be run in a conda virtual environment; it needs to be run on the main python installation. This is because the dependencies of ROS, LangSAM and it’s dependencies have conflicting versions when installed in a conda environment. Therefore it is best to just install LangSAM into the main python installation.

# Running in a Docker Container
## Building and Running Locally
This project can be built inside of a Docker container using the included Dockerfiles. These Dockerfiles build off an image created using the [jetson-containers](https://github.com/dusty-nv/jetson-containers) package. There are multiple Dockerfiles to improve modularity and speed up development. It should be possible to simply build Dockerfile.upper immediately, as a working image of Dockerfile.base is available on Docker Hub:
```bash
docker build -f Dockerfile.upper -t image_processing:latest .
```
However, if for whatever reason the Docker Hub image is not working, you can try building Dockerfile.base locally and then building Dockerfile.upper on top of that:
```bash
docker build -f Dockerfile.base -t image_processing_base:latest .
```
The first line in Dockerfile.upper can then be replaced with: ```FROM image_processing_base:latest```

**Note:** To build the Dockerfiles on x86 architecture, [this](https://www.stereolabs.com/docs/docker/building-arm-container-on-x86) guide is helpful.

To run the container after building a local image and to ensure it is connected to the other ROS2 nodes running on the system (either in other containers or natively) run the following:
```bash
docker run -it --network="host" image_processing:latest
```

## Running Directly From Docker Hub
The Docker Image can be pulled directly from Docker Hub and run using the following command:
```bash
docker pull andors/image_processing:latest
docker run -it --network="host" andors/image_processing:latest
```

# Notes

- The Dockerfiles in this repository were written to run on a Nvidia Jetson Orin AGX. This means they build on ARM architecture (not x86). For more details on how to build (including how to build using an x86 machine), see the Docker section above.
- For more information about each program, see the README file in the image_processing folder.
- For easiest operation, use the launch files provided in the "launch" directory. Some of these also have parameters which can be changed to modify their behaviours. For more information see the launch files.

# For Remote Development using VSCode (useful for Jetsons)
1. Install Remote Development extensions:
- Remote - SSH
- Remote - Containers
2. Set up Remote -SSH in VSCode
    1. Open VSCode.
    2. Press Ctrl+Shift+P to open the Command Palette.
    3. Type "Remote-SSH: Connect to Host..." and select it.
    4. Enter the SSH connection string for your Jetson Orin AGX (e.g., your_username@jetson_ip_address).
3. Open a remote folder by clicking the Explorer icon on the sidebar and selecting "Open Folder", then browsing to the desired folder on the Jetson.