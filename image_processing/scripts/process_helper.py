# Authored by: Andor Siegers

import rospy
from geometry_msgs.msg import Point
from image_processing.msg import MaskArray, MaskData
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from scipy.ndimage import measurements, label
import cv2

# --------------------------------------------- Conversion functions -------------------------------------------------
def convert_to_MaskArray(centroids_as_pixels, depth_vals, phrases, logits, masks_np, rgb_image, depth_image):
    bridge = CvBridge()
    mask_array = MaskArray()
    for centroid, depth_val, phrase, logit, mask in zip(centroids_as_pixels, depth_vals, phrases, logits, masks_np):
        temp_mask = MaskData()
        temp_mask.phrase = phrase
        temp_mask.centroid = Point(centroid[0], centroid[1], depth_val) # the z part of the point is depth (in mm)
                                                                        # this should not be confused for a point in
                                                                        # 3d cartesian space, as x and y are in px
        temp_mask.logit = logit
        mask_image = (mask * 255).astype(np.uint8)
        temp_mask.mask = bridge.cv2_to_imgmsg(mask_image, encoding="mono8") 

        mask_array.mask_data.append(temp_mask)
    # Add rgb image to array data
    mask_array.rgb_img = bridge.cv2_to_imgmsg(rgb_image, encoding="bgr8")
    mask_array.depth_img = bridge.cv2_to_imgmsg(depth_image, encoding="32FC1")
    return mask_array

def convert_mask_data(mask_data):
    try:
        bridge = CvBridge()

        # Empty arrays
        phrases = []
        centroids = []
        depth_vals = []
        logits = []
        masks = []  # boolean array
        mask_image = None  # RGB image accompanying mask data
        depth_image = None # Depth image accompanying mask data

        # Convert data back to accessible data types
        for temp_mask in mask_data.mask_data:
            temp_phrase = temp_mask.phrase
            centroid_loc = (int(temp_mask.centroid.x), int(temp_mask.centroid.y))  # Convert to integer
            depth_val = temp_mask.centroid.z / 1000
            temp_logit = temp_mask.logit

            mask_img = bridge.imgmsg_to_cv2(temp_mask.mask, desired_encoding="mono8")
            mask_img = (mask_img / 255).astype(bool)  # Convert back to boolean array
            
            phrases.append(temp_phrase)
            centroids.append(centroid_loc)
            depth_vals.append(depth_val)
            logits.append(temp_logit)
            masks.append(mask_img)

        # Convert rgb image to OpenCV format
        mask_image = bridge.imgmsg_to_cv2(mask_data.rgb_img, desired_encoding="bgr8")
        depth_image = bridge.imgmsg_to_cv2(mask_data.depth_img, desired_encoding="32FC1")

        return phrases, centroids, depth_val, logits, masks, mask_image, depth_image

    except CvBridgeError as e:
        rospy.logerr(f"CvBridge Error: {e}")
    except Exception as e:
        rospy.logerr(f"Error processing mask data: {e}")

# --------------------------------------------- Conversion functions -------------------------------------------------

# ------------------------------------------------ Mask Processing ---------------------------------------------------
def split_mask(mask, min_connection_width=5):
    # Dilate the mask to remove narrow connections
    kernel = np.ones((min_connection_width, min_connection_width), np.uint8)
    dilated_mask = cv2.dilate(mask.astype(np.uint8), kernel, iterations=1)
    
    # Label connected components in the dilated mask
    labeled_mask, num_labels = label(dilated_mask)
    
    # Initialize a list to store the separated masks
    separated_masks = []
    
    for label_index in range(1, num_labels + 1):
        # Create a mask for the current component
        component_mask = (labeled_mask == label_index)
        
        # Erode back the component mask to restore its original shape
        eroded_mask = cv2.erode(component_mask.astype(np.uint8), kernel, iterations=1)
        
        # Add the eroded mask to the list
        separated_masks.append(eroded_mask.astype(bool))
    
    return separated_masks

# ------------------------------------------------ Mask Processing ---------------------------------------------------

# -------------------------------------------- Centroid calculation --------------------------------------------------
def calculate_centroids(masks_np):
    # Calculates each centroid in a list of masks
    centroids = []
    for m in masks_np:
        centroid = measurements.center_of_mass(m)
        centroids.append(centroid)

    # convert y,x to x,y
    centroids_as_pixels = [(int(x), int(y)) for y, x in centroids]

    return centroids_as_pixels

# -------------------------------------------- Centroid calculation --------------------------------------------------

# ---------------------------------------------- Depth calculation ---------------------------------------------------
def map_rgb_to_depth(rgb_x, rgb_y, depth_image, RGB_INTRINSICS, DEPTH_INTRINSICS, DEPTH_DIST_COEFFS, EXTRINSIC_MATRIX):
        # Map an RGB pixel to the corresponding depth pixel and retrieve the depth value.
        # Check if RGB pixel is within image bounds
        if rgb_x < 0 or rgb_x >= 1920 or rgb_y < 0 or rgb_y >= 1080:
            rospy.logerr("RGB pixel is out of bounds.")
            return -1
        
        # Step 1: Normalize the RGB pixel coordinates
        normalized_rgb_point = np.linalg.inv(RGB_INTRINSICS).dot([rgb_x, rgb_y, 1])
        
        # Step 2: Transform the normalized point to the depth camera coordinate system
        point_3d_rgb = np.array([normalized_rgb_point[0], normalized_rgb_point[1], 1, 1])
        point_3d_depth = np.linalg.inv(EXTRINSIC_MATRIX).dot(point_3d_rgb)
        point_3d_depth /= point_3d_depth[3]  # Normalize homogeneous coordinates
        
        # Step 3: Project the 3D point to the 2D depth image plane
        x_d = (point_3d_depth[0] * DEPTH_INTRINSICS[0, 0] / point_3d_depth[2]) + DEPTH_INTRINSICS[0, 2]
        y_d = (point_3d_depth[1] * DEPTH_INTRINSICS[1, 1] / point_3d_depth[2]) + DEPTH_INTRINSICS[1, 2]
        
        # Step 4: Undistort the depth point
        undistorted_depth_point = cv2.undistortPoints(np.array([[x_d, y_d]], dtype=np.float32), DEPTH_INTRINSICS, DEPTH_DIST_COEFFS, P=DEPTH_INTRINSICS)
        depth_x, depth_y = undistorted_depth_point[0, 0]
        
        # Step 5: Retrieve the depth value
        if 0 <= int(depth_x) < depth_image.shape[1] and 0 <= int(depth_y) < depth_image.shape[0]:
            depth_value = depth_image[int(depth_y), int(depth_x)]
            return depth_value
        else:
            rospy.logerr("The computed depth pixel is out of bounds.")
            return -1

def map_depth_to_rgb(depth_x, depth_y, depth_value, RGB_INTRINSICS, DEPTH_INTRINSICS, DEPTH_DIST_COEFFS, EXTRINSIC_MATRIX):
    # NOTE: this does not use undistortion, as I could not figure out how to get RGB distortion coefficients. This should be looked into further TODO

    # Step 1: Normalize the depth pixel coordinates
    normalized_depth_point = np.linalg.inv(DEPTH_INTRINSICS).dot([depth_x * depth_value, depth_y * depth_value, depth_value])

    # Step 2: Transform the normalized point to the RGB camera coordinate system
    point_3d_depth = np.array([normalized_depth_point[0], normalized_depth_point[1], normalized_depth_point[2], 1])
    point_3d_rgb = EXTRINSIC_MATRIX.dot(point_3d_depth)
    point_3d_rgb /= point_3d_rgb[3]  # Normalize homogeneous coordinates

    # Step 3: Project the 3D point to the 2D RGB image plane
    x_rgb = (point_3d_rgb[0] * RGB_INTRINSICS[0, 0] / point_3d_rgb[2]) + RGB_INTRINSICS[0, 2]
    y_rgb = (point_3d_rgb[1] * RGB_INTRINSICS[1, 1] / point_3d_rgb[2]) + RGB_INTRINSICS[1, 2]

    # Step 4: Undistort the RGB point
    undistorted_rgb_point = cv2.undistortPoints(np.array([[x_rgb, y_rgb]], dtype=np.float32), RGB_INTRINSICS, None, P=RGB_INTRINSICS)
    rgb_x, rgb_y = undistorted_rgb_point[0, 0]
    
    return int(rgb_x), int(rgb_y)


def calculate_average_depths_from_rgb_mask(masks, depth_image, RGB_INTRINSICS, DEPTH_INTRINSICS, DEPTH_DIST_COEFFS, EXTRINSIC_MATRIX):
    # Calculate the average depth in each mask
    average_depths = []
    for mask in masks:
        # Get the coordinates of the unmasked pixels
        unmasked_coords = np.argwhere(mask)
        if len(unmasked_coords) == 0:
            average_depths.append(None)
            continue
        
        # Normalize the RGB pixel coordinates
        normalized_rgb_points = np.linalg.inv(RGB_INTRINSICS).dot(
            np.vstack((unmasked_coords[:, 1], unmasked_coords[:, 0], np.ones(unmasked_coords.shape[0])))
        )

        # Transform the normalized points to the depth camera coordinate system
        points_3d_rgb = np.vstack((normalized_rgb_points[0], normalized_rgb_points[1], np.ones(normalized_rgb_points.shape[1]), np.ones(normalized_rgb_points.shape[1])))
        points_3d_depth = np.linalg.inv(EXTRINSIC_MATRIX).dot(points_3d_rgb)
        points_3d_depth /= points_3d_depth[3]  # Normalize homogeneous coordinates

        # Project the 3D points to the 2D depth image plane
        x_d = (points_3d_depth[0] * DEPTH_INTRINSICS[0, 0] / points_3d_depth[2]) + DEPTH_INTRINSICS[0, 2]
        y_d = (points_3d_depth[1] * DEPTH_INTRINSICS[1, 1] / points_3d_depth[2]) + DEPTH_INTRINSICS[1, 2]

        # Undistort the depth points
        depth_points = np.vstack((x_d, y_d)).T
        undistorted_depth_points = cv2.undistortPoints(np.array([depth_points], dtype=np.float32), DEPTH_INTRINSICS, DEPTH_DIST_COEFFS, P=DEPTH_INTRINSICS)
        undistorted_depth_points = undistorted_depth_points[0]

        # Retrieve the depth values
        depth_values = []
        for x, y in undistorted_depth_points:
            if 0 <= int(x) < depth_image.shape[1] and 0 <= int(y) < depth_image.shape[0]:
                depth_values.append(depth_image[int(y), int(x)])
        
        # Calculate the average depth
        if depth_values:
            average_depth = np.mean(depth_values)
            average_depths.append(average_depth)
        else:
            average_depths.append(None)

    return average_depths
# ---------------------------------------------- Depth calculation ---------------------------------------------------

# ------------------------------------------------ Target finding ----------------------------------------------------
def find_target(target_phrase, target_confidence_threshold, centroids_as_pixels, phrases, depth_vals, logits):
    # Selects which object to target. This can be implemented in different ways,
    # but for now, it will just select the closest object.
    # Find minimum depth of nearest target
    target_x = -1
    target_y = -1
    target_depth = -1 # TODO if target depth is 0 (object not in detection range), it will be selected. This can cause issues, especially if the target is outside of the detection range.
    min_depth = 100000
    for temp_centroid, temp_phrase, temp_depth, temp_logit in zip(centroids_as_pixels, phrases, depth_vals, logits):
        if target_phrase in temp_phrase and temp_depth < min_depth and temp_logit > target_confidence_threshold:
            target_x = temp_centroid[0]
            target_y = temp_centroid[1]
            target_depth = temp_depth
            min_depth = temp_depth
    target = Point(target_x, target_y, target_depth) # Note: z in value is in mm, while x and y are in px
    return target
# ------------------------------------------------ Target finding ----------------------------------------------------