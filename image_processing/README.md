## Introduction
This file is meant to describe the features of each script in the [image_processing](image_processing) and each launchfile in the [launch](launch) folder.

# Scripts

## Image Display Program
> [!WARNING]
> This program is not yet implemented on the Jetson.

### Program Description
[`img_display_node.py`](image_processing/image_display_node.py) simply displays the raw images output by the RGBD camera. This is helpful for debugging and developing new programs, as it is taken directly from the camera without any further processing.

> [!NOTE]
> This program is not necessary anymore, as RVIZ can be used to visualize image topics. However, it continues to be a part of this package to demonstrate how image topics can be subscribed to and processed.

### ROS Inputs
| Input | Topic | Datatype | Description |
| --- | --- | --- | --- |
| RGB Image | `zed/zed_node/rgb/image_rect_color` | `sensor_msgs/Image` (bgra8 encoding) | Rectified RGB image from the RGBD camera. See [here](https://www.stereolabs.com/docs/ros2/zed-node) for topic description. |
| Depth Image | `zed/zed_node/depth/depth_registered` | `sensor_msgs/Image` (32FC1 encoding) | Registered depth image from the RGBD camera. |

## Image Saving Program
> [!WARNING]
> This program is not yet implemented on the Jetson.

### Program Description
[`img_save_node.py`](image_processing/image_save_node.py) saves the raw images received from the RGBD camera, as well as the mask images to a specified folder on the computer. The adjustment of various program parameters is enabled through the manipulation of the constants defined at the top of the file. These include the save directory, folder name, save rate, and a variety of other constants that determine whether each image type is saved or not. For example, if a test is being run where only RGB images need to be saved, the [`SAVE_RGB`](image_processing/image_save_node.py#L19) constant can be set to true, while all other save selection constants can be set to false. This allows for flexibility during testing and ensures that only important data is saved.

> [!CAUTION]
> If the directory in which files are being saved already exists, this program will clear it before starting the saving process. This enables continued testing without filling up the computer’s disk space. To save the image files of a test permanently, they must either be moved or copied to a separate folder outside of the save folder. Alternatively, the [`SAVE_FOLDER_NAME`](image_processing/image_save_node.py#L10) and/or [`SAVE_PATH`](image_processing/image_save_node.py#L7) constants can be changed, leaving the previously saved data intact and creating a new directory to save image data to upon program startup.

### ROS Inputs
| Input | Topic | Datatype | Description |
| --- | --- | --- | --- |
| RGB Image | `zed/zed_node/rgb/image_rect_color` | `sensor_msgs/Image` (bgra8 encoding) | Rectified RGB image from the RGBD camera. See [here](https://www.stereolabs.com/docs/ros2/zed-node) for topic description.
| Depth Image | `zed/zed_node/depth/depth_registered` | `sensor_msgs/Image` (32FC1 encoding) | Registered depth image from the RGBD camera. |
| Mask Image | `/process/mask_img` | `sensor_msgs/Image` (rgb8 encoding) | RGB image with object masks overlayed (output by processing program). |

## Image Processing Program

### Program Description
[`img_processor_node.py`](image_processing/image_processor_node.py) processes the RGB images it receives from the RGBD camera and publishes the cartesian coordinates of the specified target prompt to a ROS topic. It feeds the raw RGB images to an image recognition model along with a text prompt. This then calculates a mask over the image for each tracked object, which it then finds the centroid of. It then uses the depth data taken from the camera to calculate the cartesian coordinates of the centroid, along with the average depth over the entire object mask. It publishes all this data to a ROS topic using a custom msg structure (defined in the [process_msgs](/process_msgs) package). The program then compares the calculated mask phrases with the given target phrase and sends the cartesian coordinates of the target centroid to another ROS topic. If two or more targets are detected, the program sends the coordinates of the target with the lowest average depth. This can be modified in the future if a different, more complex targeting algorithm is desired.

This program features a variety of ROS parameters that can be modified by changing their defaults within the code itself or by using a launchfile. ALternatively, they can be specified when launching the node directly from the terminal. The following table gives a list of parameters:

### Parameter Table
| Name | Datatype | Description | Example Value |
| --- | --- | --- | --- |
| `prompt` | `string` | The prompt given to the AI model for image detection. Multiple prompts can be used by separating each orompt with a period. | 'person.phone' |
| `target` | `string` | The program selects a mask corresponding this target phrase to publish as the navigation goal. It **must** be contained in the prompt string. | 'person' |
| `target_confidence_threshold` | `double` | The minimum confidence value required for an object mask to be considered as a target (ranges from 0.0-0.99). | 0.5 |
| `print_output` | `boolean` | Determines whether the program prints information about each processed frame to the console (i.e. processing time, masks detected, the locations of their centroids, etc.). | False |
| `clear_output` | `boolean` | Determines whether the program clears the terminal before printing more information. | True |
| `regulate_process_rate` | `boolean` | Determines whether the processing rate should be regulated, or whether it should process as fast as possible. | False |
| `processing_rate` | `double` | Maximum processing frequency (in Hz). Only relevant if `regulate_process_frequency` is `True`. It should be noted that each processing cycle will take at minimum the amount of time needed to process one image, regardless of the set rate. If processing rate is 0, the rate will not be regulated. | 1.0 |
| `rgb_img_topic` | `string` | The ROS topic on which the RGB images from the RGBD camera are published. | 'zed/zed_node/rgb/image_rect_color' |
| `depth_img_topic` | `string` | The ROS topic on which the depth images from the RGBD camera are published. | 'zed/zed_node/depth/depth_registered' |
| `cam_info_topic` | `string` | The ROS topic on which the camera information (i.e. the instrinsic data) from the RGBD camera is published. | 'zed/zed_node/depth/camera_info' |

> [!NOTE]
> Changing the `prompt` parameter will change what the image recognition model will detect. This can be utilized to detect multiple objects in the image. The `target` parameter should contain only **one** object phrase, and **must** be contained within the prompt. This allows for detection of multiple objects, however the program sends the information of only one target to the robot control algorithm. This allows for some flexibility if multiple different kinds of objects need to be detected in the future.

### ROS Inputs
| Input | Topic | Datatype | Description |
| --- | --- | --- | --- |
| RGB Image | Defined by ROS parameter (see [parameter table](#parameter-table)) | sensor_msgs/Image (bgra8 encoding) | Rectified RGB image from the RGBD camera. See [here](https://www.stereolabs.com/docs/ros2/zed-node) for topic description.
| Depth Image | Defined by ROS parameter (see [parameter table](#parameter-table)) | sensor_msgs/Image (32FC1 encoding) | Registered depth image from the RGBD camera. |
| Camera Info | Defined by ROS parameter (see [param table](#parameter-table)) | sensor_msgs/CameraInfo | Contains camera instrinsics information. This is needed to calculate the position of a pixel in 3D cartesian space. See [here](https://docs.ros2.org/latest/api/sensor_msgs/msg/CameraInfo.html) for details about this message type. |

### ROS Outputs
| Output | Topic | Datatype | Description |
| --- | --- | --- | --- |
| Mask Data | `process/mask_data` | Custom [`MaskArray`](/process_msgs/msg/MaskArray.msg) datatype | Contains an array of [`MaskData`](/process_msgs/msg/MaskData.msg) messages, along with the corresponding RGB and depth images. Used to send output of processing program to various other programs. |
| Target Location | `process/target` | `geometry_msgs/Point` | Contains the location of the target in cartesian space, with the centroid of the RGBD camera being the origin and the Z axis extending out from the camera, perpendicular to the front face. |


## Mask Display Program
> [!WARNING]
> This program is not yet implemented on the Jetson.

### Program Description
[`mask_display_node.py`](image_processing/mask_display_node.py) receives the mask data and displays it using the OpenCV python package. It does this by overlaying each mask in green over the base RGB image. It then draws each centroid over those masks with accompanying text that describes the phrase (or object detected by the program) and the depth of the centroid and/or the average depth of the mask (in meters). This program also publishes the final image to a ROS topic, which is read by the image saving program.

> [!NOTE]
> The mask display program relies on data published by the processing program, so should only be run after the processing program has started. Using the [launchfiles](launch/) is recommended.

### ROS Inputs
| Input | Topic | Datatype | Description |
| --- | --- | --- | --- |
| Mask Data | `process/mask_data` | Custom [`MaskArray`](/process_msgs/msg/MaskArray.msg) datatype | Contains an array of [`MaskData`](/process_msgs/msg/MaskData.msg) messages, along with the corresponding RGB and depth images. For more information see [here](/process_msgs/README.md). |

### ROS Outputs
| Output | Topic | Datatype | Description |
| --- | --- | --- | --- |
| Mask Image | `/process/mask_img` | `sensor_msgs/Image` (rgb8 encoding) | RGB image with object masks overlayed (output by processing program). For more information see [here](/process_msgs/README.md). |


## Control Demo Program

### Program Description
[`ranger_control_demo.py`](image_processing/ranger_control_demo.py) is a simple program created to demonstrate and test the [image processing program](image_processing/image_processor_node.py)'s targetting functionality. This program drives the robot towards the target after receiving its location from the [image processing program](image_processing/image_processor_node.py). If the target is not within the angular tolerance (the angle spanning equally out from the robot's centerline in the horizontal plane, specified as a ROS parameter), the robot will turn towards the target until it is within the angular tolerance. Additionally, if the target is not within the following distance (distance from the RGBD camera to the target, also specified by a ROS parameter) the robot will drive forward until the target is within the following distance threshold. Both of these thresholds are visualized in the image below, along with the intended robot behaviour within and outside of those thresholds. The maximum linear and angular velocities of the robot can be specified through the launchfile or terminal, as they are also defined as ROS parameters.

![Robot threshold visualization](docs/threshold_visualization.png)

### Parameter Table
| Name | Type | Description | Example Value |
| --- | --- | --- | --- |
| `cmd_vel_topic` | `string` | Name of the topic on which the program publishes velocity commands. | /cmd_vel |
| `target_topic` | `string` | Name of the topic on which the program receives the location of the target. | /process/target |
| `lin_vel` | `double` | Maximum linear velocity sent to the robot (in m/s). | 0.5 |
| `ang_vel` | `double` | Maximum angular velocity sent to the robot (in rad/s). | 3.0 |
| `follow_dist` | `double` | Maximum distance from the target the robot can be before it starts moving laterally (in meters). See above diagram for a visualization. | 0.7 |
| `ang_tol` | `double` | The angular window in which the target can be without the robot moving (in radians). See above diagram for a visualization. | 1.0 |

### ROS Inputs
| Input | Topic | Datatype | Description |
| --- | --- | --- | --- |
| Target Location | Defined by ROS parameter (see [parameter table](#parameter-table-1)) | `geometry_msgs/Point` | Contains the location of the target in cartesian space, with the centroid of the RGBD camera being the origin and the Z axis extending out of the camera, perpendicular to the front face. |

### ROS Outputs
| Output | Topic | Datatype | Description |
| --- | --- | --- | --- |
| Command Velocity | Defined by ROS parameter (see [parameter table](#parameter-table-1)) | `geometry_msgs/Twist` | Contains the linear and angular velocities that are sent to the robot, controlling its movement. |

## Utility Program
[`utils.py`](image_processing/utils.py) is a helper program that contains many of the calculation and evaluation functions used throughout this package. This allows for functions in the file to be used in multiple different files, increasing modularity, while also keeping the other files shorter and improving readability by consolidating subfunctions their own separate file.

## Test Programs
[`cuda_torch_test.py`](image_processing/cuda_torch_test.py) is a program that simply tests whether CUDA can be accessed using pytorch. This is important, as the AI model used in the [`image processing program`](image_processing/image_processor_node.py#L103) depends on pytorch for fast and efficient processing. Without it, this package is essentially unusable, as the processing times become unreasonably slow.

[`zed_depth_test.py`](image_processing/zed_depth_test.py) creates a simple ROS node that receives images from the ZED 2i RGBD camera and outputs the image size, as well as the depth value of the center pixel. This is useful for testing the functionality of the camera and demonstrates how the camera can be accessed using ROS topics.

# Launchfiles
## Object Detection
[`object_detection.launch.py`](launch/object_detection.launch.py) is designed to allow the user to select which programs they would like to launch dynamically. It includes boolean launch parameters that activate/deactivate the following programs:
| Program to Activate/Deactivate | Parameter Name |
| --- | --- |
| [image display](image_processing/image_display_node.py) | [`run_img_display`](launch/object_detection.launch.py#L10) |
| [image saving](image_processing/image_save_node.py) | [`run_img_save`](launch/object_detection.launch.py#L11) |
| [mask display](image_processing/mask_display_node.py) | [`run_mask_display`](launch/object_detection.launch.py#L12) |

These launch parameters can be modified by changing their default values in the launchfile itself, or they can be modified at runtime by specifying the desired parameter directly in the launch command:
```bash
ros2 launch image_processing object_detection.launch.py run_mask_display:='False'
```

This launchfile also includes ROS parameter declarations which can be used to set specific variables used in a ROS node, allowing dynamic configuration of program behaviour without having to modify the code itself. Support for these is currently only included for the [image processing](image_processing/image_processor_node.py) and [control demo](image_processing/ranger_control_demo.py) programs. For a list of parameters, their datatypes, their descriptions, and some examples, see [this table](./README.md#parameter-table).

## Object Detection Lite
[`object_detection_lite.launch.py`](launch/object_detection_lite.launch.py) is designed to be a barebones version of the [`object_detection.launch.py`](launch/object_detection.launch.py) launchfile. It simply defines a few parameters and then launches the [image processing program](image_processing/image_processor_node.py). This file is intended to be the one run when launching the navigation stack on the Ranger Mini. It can be included in whatever launchfile is eventually used to launch the target finding and navigation processes.

## Object Detection and Control
[`object_detection_and_control.launch.py`](launch/object_detection_and_control.launch.py) is designed to launch the [image processing program](image_processing/image_processor_node.py) along with the [control demonstration program](image_processing/ranger_control_demo.py). It does not launch any of the other scripts in the package, as it is meant to be a simple barebones demo.