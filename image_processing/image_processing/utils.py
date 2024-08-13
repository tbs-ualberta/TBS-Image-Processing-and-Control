# Authored by: Andor Siegers

from geometry_msgs.msg import Point
from process_msgs.msg import MaskArray, MaskData
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from scipy.ndimage import measurements
import cv2
from std_msgs.msg import Header
import math

# ---------------------------------------------- Custom Class Definitions -------------------------------------------------
class CameraParams():
    def __init__(self):
        self.height = None
        self.width = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
# -------------------------------------------------------------------------------------------------------------------------

# --------------------------------------------- Data Conversion functions -------------------------------------------------
def convert_to_MaskArray(centroids_px, centroids_cartesian, phrases, logits, avg_depths, masks, rgb_image, depth_image):
    '''
    Converts regular Python data types into a ROS2 message structure that can be published to a ROS2 topic.

    Inputs:
    - centroids_px: 2D tuple containing the position of the centroids in pixel coordinates
    - centroids_cartesian: 3D tuple containing the position of the centroids in cartesian coordinates, with the center of the camera as the origin
    - phrases: array of strings containing the phrase of each mask corresponding to what object is being masked
    - logits: array of floats containing the confidence value of each mask output by the object detection model
    - avg_depths: array of floats containing the average depths of each object calculated using the mask
    - masks: array of 2D boolean arrays representing each mask (1 for masked, 0 for not masked)
    - rgb_image: rgb image that all above values are calculated from
    - depth_image: depth image that all above values are calculated from

    Output:
    - mask_array: MaskArray custom message structure that holds all mask data
    '''
    bridge = CvBridge()
    mask_array = MaskArray()
    mask_array.header = Header()
    for centroid_px, centroid_cart, phrase, logit, avg_depth, mask in zip(centroids_px, centroids_cartesian, phrases, logits, avg_depths, masks):
        temp_mask = MaskData()
        temp_mask.header = Header()
        temp_mask.phrase = phrase
        temp_mask.centroid_px = Point(x=float(centroid_px[0]), y=float(centroid_px[1]), z=float(0))
        temp_mask.centroid = Point(x=float(centroid_cart[0]), y=float(centroid_cart[1]), z=float(centroid_cart[2]))
        temp_mask.logit = float(logit)
        temp_mask.avg_depth = float(avg_depth)
        mask_image = (mask * 255).astype(np.uint8)
        temp_mask.mask = bridge.cv2_to_imgmsg(mask_image, encoding="mono8") 

        mask_array.mask_data.append(temp_mask)
    # Add rgb image to array data
    mask_array.rgb_image = bridge.cv2_to_imgmsg(rgb_image, encoding='bgr8')
    mask_array.depth_image = bridge.cv2_to_imgmsg(depth_image, encoding='32FC1')
    return mask_array

def unpack_MaskArray(mask_array):
    '''
    Unpacks custom MaskArray message into regular Python datatypes.

    Input:
    - mask_array: MaskArray custom message structure that holds all mask data

    Output:
    - phrases: array of strings containing the phrase of each mask corresponding to what object is being masked
    - centroids_px: 2D tuple containing the position of the centroids in pixel coordinates
    - centroids_cartesian: 3D tuple containing the position of the centroids in cartesian coordinates, with the center of the camera as the origin
    - logits: array of floats containing the confidence value of each mask output by the object detection model
    - avg_depths: array of floats containing the average depths of each object calculated using the mask
    - masks: array of 2D boolean arrays representing each mask (1 for masked, 0 for not masked)
    - rgb_image: rgb image that all above values are calculated from
    - depth_image: depth image that all above values are calculated from
    '''
    try:
        bridge = CvBridge()

        # Empty arrays
        phrases = []
        centroids_px = []
        centroids_cartesian = []
        logits = []
        avg_depths = []
        masks = []  # boolean arrays

        rgb_image = bridge.imgmsg_to_cv2(mask_array.rgb_image, 'bgr8')
        depth_image = bridge.imgmsg_to_cv2(mask_array.depth_image, '32FC1')

        # Convert data back to accessible data types
        for temp_mask in mask_array.mask_data:
            temp_phrase = temp_mask.phrase
            centroid_loc_px = (int(temp_mask.centroid_px.x), int(temp_mask.centroid_px.y))  # Convert to integer
            centroid_loc_cart = (temp_mask.centroid.x, temp_mask.centroid.y, temp_mask.centroid.z)
            temp_logit = temp_mask.logit
            temp_avg_depth = temp_mask.avg_depth / 1000

            mask_img = bridge.imgmsg_to_cv2(temp_mask.mask, desired_encoding="mono8")
            mask_img = (mask_img / 255).astype(bool)  # Convert back to boolean array
            
            phrases.append(temp_phrase)
            centroids_px.append(centroid_loc_px)
            centroids_cartesian.append(centroid_loc_cart)
            logits.append(temp_logit)
            avg_depths.append(temp_avg_depth)
            masks.append(mask_img)

        return phrases, centroids_px, centroids_cartesian, logits, avg_depths, masks, rgb_image, depth_image

    except CvBridgeError as e:
        print(f"CvBridge Error: {e}")
    except Exception as e:
        print(f"Error processing mask data: {e}")

# --------------------------------------------- Data Conversion functions -------------------------------------------------

# -------------------------------------------------- Mask Processing ------------------------------------------------------
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

    # convert x,y to y,x
    # centroids_px = [(int(y), int(x)) for x, y in centroids]

    centroids_px = [(int(y), int(x)) for y, x in centroids]

    return centroids_px

def get_avg_depth(mask_np, depth_array):
    # Convert boolean mask to list of coordinates
    coordinates = np.argwhere(mask_np)

    # Use coordinate list to find corresponding depth values
    depth_vals = []
    for y, x in coordinates:
        px_depth = depth_array[y, x]
        if px_depth != float('inf') and not math.isnan(px_depth):
            # Only add valid depths to depth_vals
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

def get_point_xyz(point, depth_array, camera_params:CameraParams):
    """
    Maps a pixel (r, c) to Cartesian coordinates (x, y, z) using the depth image.

    Parameters:
    - point: Contains the row and column (r, c) of the pixel.
    - depth_array: Array of depth values corresponding to the size of the image
    - camera_params: A CameraParams object containing the camera parameters.

    Returns:
    - A tuple (x, y, z) representing the Cartesian coordinates.
    """
    r, c = point
    z = depth_array[r][c]

    x = ((r - camera_params.cx) * z) / camera_params.fx
    y = ((c - camera_params.cy) * z) / camera_params.fy

    return (x, y, z)

# -------------------------------------------- Coordinate Transformations -------------------------------------------------

# -------------------------------------------------- Target finding -------------------------------------------------------
def find_target(target_phrase, target_confidence_threshold, centroids_cartesian, phrases, avg_depths, logits):
    # Selects which object to target. This can be implemented in different ways,
    # but for now, it will just select the closest object.

    # Initialize target position values
    target_x = -1.0
    target_y = -1.0
    target_depth = -1.0

    # Records whether a target has been found
    TARGET_FOUND = False

    # Used to find the closest target
    min_depth = float('inf')

    # Loop through scanned objects and find valid targets
    for temp_centroid, temp_phrase, temp_depth, temp_logit in zip(centroids_cartesian, phrases, avg_depths, logits):
        if target_phrase in temp_phrase and temp_logit > target_confidence_threshold:
            if not TARGET_FOUND:
                # If this is the first target found, set it as the target
                target_x = float(temp_centroid[0])
                target_y = float(temp_centroid[1])
                if math.isnan(temp_depth): # If target depth is invalid, set depth -1
                    target_depth = -1.0
                else: # If target depth is valid, set depth to avg_depth and update min_depth
                    target_depth = float(temp_depth)
                    min_depth = temp_depth
                TARGET_FOUND = True
            
            elif not math.isnan(temp_depth) and temp_depth < min_depth:
                # If any more valid targets are found that are closer, set that as the target
                target_x = float(temp_centroid[0])
                target_y = float(temp_centroid[1])
                target_depth = float(temp_depth)
                min_depth = temp_depth
    target = Point(x=target_x, y=target_y, z=target_depth)  # Note: these values are in cartesian coordinates, with x and y being the position
                                                            # of the centroid and the depth being the average depth of the object.
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