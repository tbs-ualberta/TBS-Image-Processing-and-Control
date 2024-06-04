# Authored by: Andor Siegers

import rospy
from geometry_msgs.msg import Point
from img_processor.msg import MaskArray, MaskData
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from scipy.ndimage import measurements
import cv2

# ------------------------------------------ For centroid calculation ------------------------------------------------
def calculate_centroids(masks_np):
    centroids = []
    for m in masks_np:
        centroid = measurements.center_of_mass(m)
        centroids.append(centroid)

    # convert y,x to x,y
    centroids_as_pixels = [(int(x), int(y)) for y, x in centroids]

    return centroids_as_pixels

# ------------------------------------------ For centroid calculation ------------------------------------------------

# -------------------------------------------- For depth calculation -------------------------------------------------
def map_rgb_to_depth(rgb_x, rgb_y, depth_image, RGB_INTRINSICS, DEPTH_INTRINSICS, DEPTH_DIST_COEFFS, EXTRINSIC_MATRIX):
        """Map an RGB pixel to the corresponding depth pixel and retrieve the depth value."""
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
            
# -------------------------------------------- For depth calculation -------------------------------------------------

# --------------------------------------------- Conversion functions -------------------------------------------------
def convert_to_MaskArray(centroids_as_pixels, depth_vals, phrases, logits, masks_np, rgb_image):
    bridge = CvBridge() # may not work
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
    return mask_array

# --------------------------------------------- Conversion functions -------------------------------------------------

# ------------------------------------------------ Target finding ----------------------------------------------------
def find_target(target_phrase, centroids_as_pixels, phrases, depth_vals):
    # Selects which object to target. This can be implemented in different ways,
    # but for now, it will just select the closest object.
    # Find minimum depth of nearest target
    target_x = -1
    target_y = -1
    target_depth = -1
    min_depth = 100000
    for temp_centroid, temp_phrase, temp_depth in zip(centroids_as_pixels, phrases, depth_vals):
        if target_phrase in temp_phrase and temp_depth < min_depth:
            target_x = temp_centroid[0]
            target_y = temp_centroid[1]
            target_depth = temp_depth
            min_depth = temp_depth
    target = Point(target_x, target_y, target_depth) # Note: z in value is in mm, while x and y are in px
    return target
# ------------------------------------------------ Target finding ----------------------------------------------------