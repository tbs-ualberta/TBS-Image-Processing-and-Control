# Authored by: Andor Siegers

import rospy
from geometry_msgs.msg import Point
from image_processing.msg import MaskArray, MaskData
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from scipy.ndimage import measurements, label
import cv2
from kinect_pub.msg import RegistrationData

# --------------------------------------------- Data Conversion functions -------------------------------------------------
def convert_to_MaskArray(centroids_as_pixels, depth_vals, phrases, logits, masks_np, reg_data_msg):
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
    mask_array.registration_data = reg_data_msg
    return mask_array

def unpack_MaskArray(mask_data):
    try:
        bridge = CvBridge()

        # Empty arrays
        phrases = []
        centroids = []
        depth_vals = []
        logits = []
        masks = []  # boolean arrays

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

        return phrases, centroids, depth_vals, logits, masks, mask_data.registration_data

    except CvBridgeError as e:
        rospy.logerr(f"CvBridge Error: {e}")
    except Exception as e:
        rospy.logerr(f"Error processing mask data: {e}")

def convert_to_RegistrationData(rgb_image, depth_image, registered_image, bigdepth_image, colour_depth_map):
    bridge = CvBridge()
    reg_data_msg = RegistrationData()

    try:
        # Convert RGB image to ROS Image message
        rgb_msg = bridge.cv2_to_imgmsg(rgb_image, encoding="bgr8")
        reg_data_msg.rgb_image = rgb_msg

        # Convert depth image to ROS Image message
        depth_msg = bridge.cv2_to_imgmsg(depth_image, encoding="32FC1")
        reg_data_msg.depth_image = depth_msg

        # Convert registered image to ROS Image message
        registered_msg = bridge.cv2_to_imgmsg(registered_image, encoding="bgr8")
        reg_data_msg.registered_image = registered_msg

        # Convert big depth image to ROS Image message
        bigdepth_msg = bridge.cv2_to_imgmsg(bigdepth_image, encoding="32FC1")
        reg_data_msg.bigdepth_image = bigdepth_msg

        # Assign colour-depth map
        reg_data_msg.colour_depth_map = list(colour_depth_map.flatten())

        rospy.loginfo("Converted simple datatypes to RegistrationData message.")
    except CvBridgeError as e:
        rospy.logerr(f"Error converting images to ROS messages: {e}")

    return reg_data_msg

def unpack_RegistrationData(data):
    # Unpacks registration data from custom message structure
    bridge = CvBridge()
    # Convert RGB Image
    try:
        rgb_image = bridge.imgmsg_to_cv2(data.rgb_image, "bgra8")
        rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGRA2BGR)  # Convert to BGR
    except Exception as e:
        rospy.logerr("Error converting RGB image: %s", e)

    # Convert Depth Image
    try:
        depth_image = bridge.imgmsg_to_cv2(data.depth_image, "32FC1")
    except Exception as e:
        rospy.logerr("Error converting Depth image: %s", e)

    # Convert Registered Image
    try:
        registered_image = bridge.imgmsg_to_cv2(data.registered_image, "bgra8")
    except Exception as e:
        rospy.logerr("Error converting Registered image: %s", e)

    # Convert Bigdepth Image
    try:
        bigdepth_image = bridge.imgmsg_to_cv2(data.bigdepth_image, "32FC1")
        bigdepth_image = np.array(bigdepth_image.data, dtype=np.float32).reshape((1082, 1920))
    except Exception as e:
        rospy.logerr("Error converting Bigdepth image: %s", e)

    # Convert Colour Depth Map
    try:
        colour_depth_map = np.array(data.colour_depth_map, dtype=np.int32).reshape((424, 512))
    except Exception as e:
        rospy.logerr("Error converting Color Depth Map: %s", e)

    return rgb_image, depth_image, registered_image, bigdepth_image, colour_depth_map

def convert_to_matrices(camera_info):
    # Initialize intrinsic matrices
    rgb_intrinsics = np.zeros((3,3))
    depth_intrinsics = np.zeros((3,3))
    rgb_dist_coeffs = np.zeros(5)
    depth_dist_coeffs = np.zeros(5)
    extrinsic_matrix = np.zeros((3,3))

    if camera_info:
        # Construct the intrinsic matrices
        rgb_intrinsics = np.array([[camera_info.rgb_fx, 0, camera_info.rgb_cx],
                                [0, camera_info.rgb_fy, camera_info.rgb_cy],
                                [0, 0, 1]])
        
        depth_intrinsics = np.array([[camera_info.ir_fx, 0, camera_info.ir_cx],
                                    [0, camera_info.ir_fy, camera_info.ir_cy],
                                    [0, 0, 1]])
        rgb_dist_coeffs = np.array([3.823e-3, 3.149e-4, 2.332e-4, -5.152e-4]) # from: https://www.ncbi.nlm.nih.gov/pmc/articles/PMC4701245/
        depth_dist_coeffs = np.array([0.0893804, -0.272566, 0, 0, 0.0958438])
        

        # Construct extrinsic matrix
        R = np.eye(3)  # Rotation matrix (3x3)
        T = np.array([0, 0, 0])  # Translation vector (3x1)
        extrinsic_matrix = np.eye(4)
        extrinsic_matrix[:3, :3] = R
        extrinsic_matrix[:3, 3] = T

    else:
        rospy.logwarn("Unable to get camera info. Intrinsic matrices are empty.")

    return rgb_intrinsics, depth_intrinsics, rgb_dist_coeffs, depth_dist_coeffs, extrinsic_matrix

# --------------------------------------------- Data Conversion functions -------------------------------------------------

# -------------------------------------------------- Mask Processing ------------------------------------------------------
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

def is_mask_overlapping(mask, overlap_masks, OVERLAP_THRESHOLD):
    '''
    Calculates whether mask is overlaping any of overlap_masks by the overlap threshold
    mask - mask that is being checked
    overlap_masks - masks that mask cannot overlap
    OVERLAP_THRESHOLD - percentage of mask that needs to overlap before it is considered "overlapping" (0.0-1.0)
    '''
    
    for existing_mask in overlap_masks:
        overlap = np.logical_and(mask, existing_mask).sum()
        if overlap / mask.sum() >= OVERLAP_THRESHOLD:
            return True
    return False
# -------------------------------------------------- Mask Processing ------------------------------------------------------

# ----------------------------------------------- Centroid calculation ----------------------------------------------------
def calculate_centroids(masks_np):
    # Calculates each centroid in a list of masks
    centroids = []
    for m in masks_np:
        centroid = measurements.center_of_mass(m)
        centroids.append(centroid)

    # convert y,x to x,y
    centroids_as_pixels = [(int(x), int(y)) for y, x in centroids]

    return centroids_as_pixels

# ----------------------------------------------- Centroid calculation ----------------------------------------------------

# -------------------------------------------- Coordinate Transformations -------------------------------------------------
def map_depth_mask_to_rgb(depth_mask, rgb_image, depth_image, colour_depth_map):
    # Initialize mask to fill
    rgb_mask = np.zeros(rgb_image.shape, dtype=bool)

    # Get coordinates of mask in depth image space
    depth_coords = np.column_stack(np.nonzero(depth_mask))

    for coord in depth_coords:
        depth_y, depth_x = coord
        color_index = colour_depth_map[depth_y, depth_x]  # Get the corresponding index in the RGB image

        if color_index < depth_image.size:
            color_y, color_x = np.unravel_index(color_index, rgb_image.shape[:2])  # Convert linear index to 2D coordinates
            rgb_mask[color_y, color_x] = True  # Mark the corresponding position in the RGB mask

    return rgb_mask

def map_rgb_point_to_cartesian(rgb_point, bigdepth):
    


# -------------------------------------------- Coordinate Transformations -------------------------------------------------

# -------------------------------------------------- Target finding -------------------------------------------------------
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
# -------------------------------------------------- Target finding -------------------------------------------------------