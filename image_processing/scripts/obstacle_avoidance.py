#!/usr/bin/env python3
# Authored by: Andor Siegers

# Assumes that the object_detection program is detecting "floor" as it's prompt
# Integrating the target into this program is doable, but the target would have to be detected and mapped as the goal instead of an obstacle.

import rospy
from image_processing.msg import MaskArray
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
from scipy.ndimage import label
from kinect_pub.srv import GetCameraInfo
from scipy.ndimage import measurements
from process_helper import convert_mask_data

class ObstacleAvoidance:
    def __init__(self):
        self.bridge = CvBridge()

        # Initialize node
        rospy.init_node('obstacle_avoidance', anonymous=True)

        # Initialize mask data arrays
        self.phrases = []
        self.centroids = []
        self.depth_vals = []
        self.logits = []
        self.masks = [] # boolean array
        self.rgb_image = None # RGB image accompanying mask data
        self.depth_image = None # Depth image accompanying mask data

        self.obstacle_masks = [] # stores the negative masks of each detected potential obstacle
        self.obstacle_masks_avg_depth = [] # stores the average depth of each obstacle
        self.obstacle_centroids = [] # stores the centroid of each obstacle

        # Get camera instrinsic data from service
        get_camera_info = rospy.ServiceProxy('/rgbd_out/get_camera_info', GetCameraInfo)
        camera_info = get_camera_info()

        # Initialize intrinsic matrices
        self.rgb_intrinsics = np.zeros((3,3))
        self.depth_intrinsics = np.zeros((3,3))
        self.depth_dist_coeffs = np.zeros(5)
        self.extrinsic_matrix = np.zeros((3,3))
        
        if camera_info:
            # Construct the intrinsic matrices
            self.rgb_intrinsics = np.array([[camera_info.rgb_fx, 0, camera_info.rgb_cx],
                                    [0, camera_info.rgb_fy, camera_info.rgb_cy],
                                    [0, 0, 1]])
            
            self.depth_intrinsics = np.array([[camera_info.ir_fx, 0, camera_info.ir_cx],
                                        [0, camera_info.ir_fy, camera_info.ir_cy],
                                        [0, 0, 1]])
            
            # Manual input for distortion coefficients
            self.depth_dist_coeffs = np.array([0.0893804, -0.272566, 0, 0, 0.0958438])

            # Construct extrinsic matrix
            R = np.eye(3)  # Rotation matrix (3x3)
            T = np.array([0, 0, 0])  # Translation vector (3x1)
            self.extrinsic_matrix = np.eye(4)
            self.extrinsic_matrix[:3, :3] = R
            self.extrinsic_matrix[:3, 3] = T
        
        # Define subscriber for mask data
        self.mask_sub = rospy.Subscriber('/process/mask_data', MaskArray, self.mask_callback)

        # Define publishers

        rospy.loginfo("Obstacle avoidance node started")

    def mask_callback(self, mask_data):
        # Runs whenever new mask data is received

        # Convert raw mask data from message to usable datatypes
        self.phrases, self.centroids, self.depth_vals, self.logits, self.masks, self.mask_image, self.depth_image = convert_mask_data(mask_data)

        # Use masks to calculate average distance of each unmasked section
        self.find_obstacles()

        # Display obstacle image (for testing)
        self.display_obstacles()

        # Use that information to calculate cost function
        # TODO
        
    def find_obstacles(self):
        # TODO this needs to find similar value depth pixels and group them together, then output the average depth of those pixels.
        # This is necessary, because if the prompt is "floor", anything above the floor will all be considered part of the "negative mask"
        # and it will not be separated into separate objects.

        if not self.masks:
            # If mask array is empty (no prompt detected)
            rospy.loginfo("Mask array empty, no floor detected.")
            return
        
        # Combine all mask areas
        floor_mask = np.zeros_like(self.masks[0], dtype=bool)
        for mask in self.masks:
            floor_mask = np.logical_or(floor_mask, mask)

        # Invert combined mask, to get unmasked area
        unmasked_area = np.logical_not(floor_mask)

        # Split masks into seperate objects (determine different obstacles)
        self.obstacle_masks = split_mask(unmasked_area)

        # Find the average depth (or distance away from the camera) of each mask
        self.obstacle_masks_avg_depth = calculate_average_depths(
            self.obstacle_masks, self.depth_image, self.rgb_intrinsics, self.depth_intrinsics, self.depth_dist_coeffs, self.extrinsic_matrix)
        
        # Find the centroid of each obstacle mask
        self.obstacle_centroids = calculate_centroids(self.obstacle_masks)

    def display_obstacles(self):
        try:
            if self.rgb_image is None:
                rospy.logwarn("rgb_image is None, skipping display")
                return
            
            overlay = self.rgb_image.copy()
            alpha = 0.5  # Transparency factor
            
            obstacle_mask_sum = np.zeros_like(self.rgb_image)

            # Convert mask to single array. This allows for even application of colour and the image is not too darkened by multiple masks being overlaid.
            for mask in self.obstacle_masks:
                obstacle_mask_scaled = np.zeros_like(self.rgb_image)
                obstacle_mask_scaled[mask] = [0, 255, 0] # change these values to change colour
                # Combine arrays
                obstacle_mask_sum = np.maximum(obstacle_mask_sum, obstacle_mask_scaled)

            # Overlay mask on the image
            cv2.addWeighted(obstacle_mask_sum, alpha, overlay, 1 - alpha, 0, overlay)

            # Draw centroid and average depth
            for centroid, avg_depth in zip(self.obstacle_centroids, self.obstacle_masks_avg_depth):
                # Draw centroid
                cv2.circle(overlay, (centroid[0], centroid[1]), 5, (0, 0, 255), -1)  # Red color for centroid

                # Put text for average depth
                cv2.putText(overlay, str(avg_depth), (centroid[0] + 10, centroid[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

            # Display image in window
            cv2.imshow("Obstacle Masks", overlay)

            # Uncomment to check if mask is displaying properly - for testing
            # cv2.imshow("Depth Image", self.depth_image)

            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f"Error displaying obstacle masks: {e}")

    def spin(self):
        rospy.spin()

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

def calculate_average_depths(unmasked_areas, depth_image, RGB_INTRINSICS, DEPTH_INTRINSICS, DEPTH_DIST_COEFFS, EXTRINSIC_MATRIX):
    average_depths = []

    for unmasked_area in unmasked_areas:
        # Get the coordinates of the unmasked pixels
        unmasked_coords = np.argwhere(unmasked_area)
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

def calculate_centroids(masks_np):
    centroids = []
    for m in masks_np:
        centroid = measurements.center_of_mass(m)
        centroids.append(centroid)

    # convert y,x to x,y
    centroids_as_pixels = [(int(x), int(y)) for y, x in centroids]

    return centroids_as_pixels

if __name__ == '__main__':
    try:
        obstacle_avoidance = ObstacleAvoidance()
        obstacle_avoidance.spin()
    except rospy.ROSInterruptException:
        pass