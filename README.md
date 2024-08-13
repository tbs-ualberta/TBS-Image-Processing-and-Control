# Introduction

This file documents the image_processing_and_control packages written for the Indro Robotics Ranger Mini 2.0 at the Telerobotic and Biorobotic Systems Group Laboratory at the University of Alberta. The image processing is intended to run in a Docker container, as it contains various delicate dependencies that need are difficult to install properly locally.

## Assumptions

The steps in this document were done with the following assumptions:

- Operating system: Ubuntu 22
- ROS version: Humble
- Camera: ZED 2i


# Install and Run Locally (Not Recommended)

For installation steps, see both the Dockerfile.base and Dockerfile.upper Dockerfiles. These outline important installation steps to get this package running. Importantly, compatible versions of pytorch and ROS2 have to be installed for everything to work correctly, which is sometimes difficult to do. Therefore it is recommended to simply run the Docker image provided.

**Note:** this project cannot be run in a conda virtual environment; it needs to be run on the main python installation. This is because the dependencies of ROS, LangSAM and it’s dependencies have conflicting versions when installed in a conda environment. Therefore it is best to just install LangSAM into the main python installation.


# Running in a Docker Container (Recommended)

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

- For more information about each program, see the [`README.md`](image_processing/README.md) file in the image_processing folder.

- For easiest operation, use the launch files provided in the [`image_processing/launch`](image_processing/launch) directory. Some of these also have parameters which can be changed to modify their behaviours. For more information see the launch files.

- The image processing included in this package runs at around 0.5 Hz on the Nvidia Jetson in MAXN mode without other intensive programs running in parallel. In 30W mode, this is more like 0.125 Hz (7-8 seconds per cycle). This is not fast enough to update in real time, however there are a few ways around this:

    1. The image processing program could be setup to run on a separate PC with a more capable (and less power limited) GPU.

        - This would require sending the depth image stream wirelessly to another PC using ROS, which may present bandwidth challenges (among other things).

        - We may have to ask Indro for their help, as currently the ZED camera topics do not publish on computers connected remotely through ZeroTier or LAN (I don't know why). Additionally, they will have to authorize any new computers to connect to the network.

    2. The image processing program could continue to run natively, with a very slow processing rate. Once the [Navigation2 stack](https://docs.nav2.org/index.html) is implemented, the target can be found using this program, and then placed in the robot's internal map. The robot could then use this saved position and navigate to the target without having to process more images.

        - The problem with this approach is that it would not support dynamic target following, as the target would only be updated once; when the user gives it a new prompt.

        - Alternatively, the processing program could run continuously, allowing us to update the target periodically, however this may affect the performance of the navigation programs, and would still not give us perfect real time target detection (i.e. it would not be suitable for following a fast moving human through a complex environment).


# For Remote Development using VSCode (useful for Jetsons)

1. On your local VSCode installation, install the [Remote Development](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack) extension pack, which allows for remote development using your local VSCode installation.

2. (optional) Install [Remote Explorer](https://marketplace.visualstudio.com/items?itemName=ms-vscode.remote-explorer) extension, which dispays a list of remote machines available to connect to.

3. Set up Remote-SSH in VSCode
    1. Open VSCode.
    2. Open the Command Palette with `Ctrl+Shift+P`.
    3. Type `Remote-SSH: Connect to Host...` and select it.
    4. Enter the SSH connection string for your Jetson Orin AGX (e.g., `your_username@jetson_ip_address`) and enter the password to connect to it. **Note:** you may need to setup an SSH key to allow for remote connection to the Jetson. For more information on how to do that, see [this](https://code.visualstudio.com/docs/remote/ssh-tutorial) guide.

4. Open a remote folder by clicking the Explorer icon on the sidebar and selecting "Open Folder", then browsing to the desired folder on the Jetson.