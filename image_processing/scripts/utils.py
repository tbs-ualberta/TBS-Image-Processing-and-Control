# Authored by: Andor Siegers

import rospy
from geometry_msgs.msg import Point
from image_processing.msg import MaskArray, MaskData
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from scipy.ndimage import measurements, label
import cv2
from kinect_pub.msg import RegistrationData
from kinect_pub.srv import GetCameraInfo
from std_msgs.msg import Header

# --------------------------------------------- Data Conversion functions -------------------------------------------------
def convert_to_MaskArray(centroids_as_pixels, centroid_depths, phrases, logits, avg_depths, masks_np, reg_data_msg):
    bridge = CvBridge()
    mask_array = MaskArray()
    mask_array.header = Header()
    for centroid, depth_val, phrase, logit, avg_depth, mask in zip(centroids_as_pixels, centroid_depths, phrases, logits, avg_depths, masks_np):
        temp_mask = MaskData()
        temp_mask.header = Header()
        temp_mask.phrase = phrase
        temp_mask.centroid = Point(centroid[0], centroid[1], depth_val) # the z part of the point is depth (in mm)
                                                                        # this should not be confused for a point in
                                                                        # 3d cartesian space, as x and y are in px
        temp_mask.logit = logit
        temp_mask.avg_depth = avg_depth
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
        centroid_depths = []
        logits = []
        avg_depths = []
        masks = []  # boolean arrays

        # Convert data back to accessible data types
        for temp_mask in mask_data.mask_data:
            temp_phrase = temp_mask.phrase
            centroid_loc = (int(temp_mask.centroid.x), int(temp_mask.centroid.y))  # Convert to integer
            depth_val = temp_mask.centroid.z / 1000
            temp_logit = temp_mask.logit
            temp_avg_depth = temp_mask.avg_depth / 1000

            mask_img = bridge.imgmsg_to_cv2(temp_mask.mask, desired_encoding="mono8")
            mask_img = (mask_img / 255).astype(bool)  # Convert back to boolean array
            
            phrases.append(temp_phrase)
            centroids.append(centroid_loc)
            centroid_depths.append(depth_val)
            logits.append(temp_logit)
            avg_depths.append(temp_avg_depth)
            masks.append(mask_img)

        return phrases, centroids, centroid_depths, logits, avg_depths, masks, mask_data.registration_data

    except CvBridgeError as e:
        rospy.logerr(f"CvBridge Error: {e}")
    except Exception as e:
        rospy.logerr(f"Error processing mask data: {e}")

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

    # Convert Undistorted Image
    try:
        undistorted_image = bridge.imgmsg_to_cv2(data.undistorted_image, "32FC1")
    except Exception as e:
        rospy.logerr("Error converting Undistorted image: %s", e)

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

    # Convert Process Start Timestamp
    try:
        start_time = data.process_start_time
    except Exception as e:
        rospy.logerr("Error converting Color Depth Map: %s", e)

    return rgb_image, depth_image, undistorted_image, registered_image, bigdepth_image, colour_depth_map, start_time

def get_camera_parameters():
    rospy.wait_for_service('/rgbd_out/get_camera_info')
    try:
        get_camera_info = rospy.ServiceProxy('/rgbd_out/get_camera_info', GetCameraInfo)
        response = get_camera_info()
        
        # Construct the dictionary with depth camera parameters
        depth_params = {
            'cx': response.ir_cx,
            'cy': response.ir_cy,
            'fx': response.ir_fx,
            'fy': response.ir_fy
        }
        
        # Optional: Also get RGB camera parameters if needed
        rgb_params = {
            'cx': response.rgb_cx,
            'cy': response.rgb_cy,
            'fx': response.rgb_fx,
            'fy': response.rgb_fy
        }
        
        return depth_params, rgb_params
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return None, None

def convert_to_matrices(camera_info): # This function is not used, but could be useful in the future
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
    Calculates whether mask is overlaping any of overlap_masks by the overlap threshold.
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

def get_avg_depth(mask_np, bigdepth):
    # Convert boolean mask to list of coordinates
    coordinates = np.argwhere(mask_np)

    # Use coordinate list to find corresponding depth values
    depth_vals = []
    for y, x in coordinates:
        px_depth = bigdepth[y+1, x]
        if px_depth != float('inf'):
            depth_vals.append(px_depth)

    # Calculate average of depth values
    avg_depth = np.mean(depth_vals)

    return avg_depth

# ----------------------------------------------- Centroid calculation ----------------------------------------------------

# -------------------------------------------- Coordinate Transformations -------------------------------------------------
def map_depth_mask_to_rgb(depth_mask, rgb_image, depth_image, colour_depth_map):
    '''
    Takes a boolean 2D array representing a mask over a depth image and converts it to RGB space, so it can be applied to the corresponding RGB image.
    Input:
    depth_mask - 2D boolean array
    rgb_image - RGB image (this is only needed for shaping the image array)
    depth_image - Depth image (only needed for image size)
    colour_depth_map - integer array that maps the corresponding RGB pixel for each depth pixel
    '''
    # Initialize mask to fill
    rgb_mask = np.zeros(rgb_image.shape[:2], dtype=bool)

    # Get coordinates of mask in depth image space
    print("Number of true values in depth mask: " + str((depth_mask == True).sum()))
    depth_coords = np.column_stack(np.nonzero(depth_mask))
    # print(depth_coords)

    for coord in depth_coords:
        depth_y, depth_x = coord
        color_index = colour_depth_map[depth_y, depth_x]  # Get the corresponding index in the RGB image

        if color_index < depth_image.size and color_index != -1:
            color_y, color_x = np.unravel_index(color_index, rgb_image.shape[:2])  # Convert linear index to 2D coordinates
            rgb_mask[color_y, color_x] = True  # Mark the corresponding position in the RGB mask

    return rgb_mask

def get_point_xyz(undistorted, point, depth_params):
    """
    Maps a pixel (r, c) to Cartesian coordinates (x, y, z) using the undistorted depth image.

    Parameters:
    - undistorted: The undistorted depth image as a 2D numpy array.
    - r, c: The row and column of the pixel.
    - depth_params: A dictionary containing the depth camera's intrinsic parameters:
        {
            'cx': cx,
            'cy': cy,
            'fx': fx,
            'fy': fy
        }

    Returns:
    - A tuple (x, y, z) representing the Cartesian coordinates.

    This function is a conversion of the getPointXYZ C++ function in the libfreenect2 package
    See here: https://github.com/OpenKinect/libfreenect2/blob/fd64c5d9b214df6f6a55b4419357e51083f15d93/src/registration.cpp#L342
    """
    r = point[0]
    c = point[1]
    bad_point = np.nan
    cx = depth_params['cx']
    cy = depth_params['cy']
    fx = 1 / depth_params['fx']
    fy = 1 / depth_params['fy']

    depth_val = undistorted[r, c] / 1000.0  # scaling factor to convert to meters

    if np.isnan(depth_val) or depth_val <= 0.001:
        # depth value is not valid
        x = y = z = bad_point
    else:
        x = (c + 0.5 - cx) * fx * depth_val
        y = (r + 0.5 - cy) * fy * depth_val
        z = depth_val

    # Convert to mm
    x = x / 1000
    y = y / 1000
    z = z / 1000
    return x, y, z
# -------------------------------------------- Coordinate Transformations -------------------------------------------------

# -------------------------------------------------- Target finding -------------------------------------------------------
def find_target(target_phrase, target_confidence_threshold, centroids_as_pixels, phrases, object_depths, logits):
    # Selects which object to target. This can be implemented in different ways,
    # but for now, it will just select the closest object.
    # Find minimum depth of nearest target
    target_x = -1
    target_y = -1
    target_depth = -1 # TODO if target depth is 0 (object not in detection range), it will be selected. This can cause issues, especially if the target is outside of the detection range.
    min_depth = float('inf')
    for temp_centroid, temp_phrase, temp_depth, temp_logit in zip(centroids_as_pixels, phrases, object_depths, logits):
        if target_phrase in temp_phrase and temp_depth < min_depth and temp_logit > target_confidence_threshold:
            target_x = temp_centroid[0]
            target_y = temp_centroid[1]
            target_depth = temp_depth
            min_depth = temp_depth
    target = Point(target_x, target_y, target_depth) # Note: z in value is in mm, while x and y are in px
    return target
# -------------------------------------------------- Target finding -------------------------------------------------------

# ------------------------------------------------- Display functions -----------------------------------------------------

def display_depth_with_masks(depth_image, masks):
    """
    Normalize the depth image and display the masks over it in green.

    :param depth_image: 2D numpy array representing the raw depth image
    :param masks: list of 2D numpy arrays representing the masks
    """
    # Normalize the depth image to the range [0, 255]
    depth_image_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
    depth_image_normalized = np.uint8(depth_image_normalized)

    # Convert the normalized depth image to a 3-channel BGR image
    depth_image_bgr = cv2.cvtColor(depth_image_normalized, cv2.COLOR_GRAY2BGR)
    
    overlay = np.zeros_like(depth_image_bgr)

    # Overlay each mask in green
    for mask in masks:
        temp_overlay = np.zeros_like(depth_image_bgr)
        temp_overlay[mask] = [0, 255, 0]
        overlay = np.maximum(overlay, temp_overlay)

    depth_image_bgr = cv2.addWeighted(depth_image_bgr, 1, overlay, 0.5, 0)

    # Display the result
    cv2.imshow('Depth Image with Masks', depth_image_bgr)
    cv2.waitKey(1)  # Update the display

# Example usage:
# depth_image = np.random.randint(0, 65535, size=(480, 640), dtype=np.uint16)
# masks = [np.random.randint(0, 2, size=(480, 640), dtype=np.uint8) for _ in range(3)]
# display_depth_with_masks(depth_image, masks)

# ------------------------------------------------- Display functions -----------------------------------------------------