## Introduction
This package contains the custom message structures needed in the `image_processing` package to hold and transfer mask data.

## MaskData
[`MaskData`](msg/MaskData.msg) contains all the important information calculated from one image mask. Each component of this custom message is described below:

| Name | Datatype | Description |
| --- | --- | --- |
| header | `std_msgs/Header` | The standard ROS2 header that includes things like a timestamp.
| phrase | `string` | The phrase associated with the mask. This is essentially the description of the object being masked. |
| centroid_px | `geometry_msgs/Point` | The 2D pixel coordinates of the mask's centroid. Given as a point datatype, but z value is always 0. |
| centroid | `geometry_msgs/Point` | The 3D cartesian coordinates of the mask's centroid (in meters), with the RGBD camera as the origin. |
| logit | `float32` | The confidence value of the model that the mask is indeed correlated with the phrase (0-0.99). |
| avg_depth | `float32` | The average depth calculated over the entire mask. This throws out pixels that are invalid, and only uses valid, in range pixels for calculation.
| mask | `sensor_msgs/Image` (mono8 encoding) | A boolean image that corresponds to the size of the RGB image. When overlayed, masks the object. This is the image from which the centroid location is calculated.


## MaskArray
[`MaskArray.msg`](msg/MaskArray.msg) contains an array of [`MaskData`](msg/MaskData.msg) messages, while also including the RGB and Depth images those values were calculated from. This allows all calculated data to always stay together, building from previous data. This is very important for synchronization of data, especially when displaying mask overlays, as the overlay should display over the image that was used to calculate it, not necessarily over the newest image available. Each component of this custom message is described below:

| Name | Datatype | Description |
| --- | --- | --- |
| header | `std_msgs/Header` | The standard ROS2 header that includes things like a timestamp.
| mask_data | `process_msgs/MaskData` | An array of [`MaskData`](#maskdata) messages, each corresponding to a separate mask over the same image. |
| rgb_image | `sensor_msgs/Image` (bgra8 encoding) | The RGB image used to calculate each mask.
| depth_image | | `sensor_msgs/Image` (32FC1 encoding) | The depth image used to calculate the cartesian coordinates of each mask centroid, as well as the average depth. |