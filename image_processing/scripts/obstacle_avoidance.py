#!/usr/bin/env python3
# Authored by: Andor Siegers

# Assumes that the object_detection program is detecting "floor" as it's prompt
# Integrating the target into this program is doable, but the target would have to be detected and mapped as the goal instead of an obstacle.

import rospy
from image_processing.msg import MaskArray
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
from kinect_pub.srv import GetCameraInfo
from scipy.ndimage import measurements
from process_helper import unpack_MaskArray, calculate_centroids, calculate_average_depths_from_rgb_mask, split_mask, convert_to_matrices
from sklearn.cluster import DBSCAN

# ----------------------------------------------------- Constants -------------------------------------------------------
EPS = 0.05 # for DBSCAN - maximum distance between two points for them to be considered as neighbors (in mm)

MIN_SAMPLES = 10 # for DBSCAN - the minimum number of points required to form a mask
# -----------------------------------------------------------------------------------------------------------------------

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
        self.rgb_dist_coeffs = np.zeros(5)
        self.depth_dist_coeffs = np.zeros(5)
        self.extrinsic_matrix = np.zeros((3,3))
        
        # Convert intrinsic data to matrices for later use
        self.rgb_intrinsics, self.depth_intrinsics, self.rgb_dist_coeffs, self.depth_dist_coeffs, self.extrinsic_matrix = convert_to_matrices(camera_info)
        
        # Define subscriber for mask data
        self.mask_sub = rospy.Subscriber('/process/mask_data', MaskArray, self.mask_callback)

        # Define publishers

        rospy.loginfo("Obstacle avoidance node started")

    def mask_callback(self, mask_data):
        # Runs whenever new mask data is received

        # Convert raw mask data from message to usable datatypes
        self.phrases, self.centroids, self.depth_vals, self.logits, self.masks, self.mask_image, self.depth_image = unpack_MaskArray(mask_data)

        # Use masks to calculate average distance of each unmasked section
        self.find_obstacles()

        # Display obstacle image (for testing)
        self.display_obstacles()

        # Use that information to calculate cost function
        # TODO build a map of the environment and use potential field path planning to find a path
        
    def find_obstacles(self):
        # TODO this needs to find similar value depth pixels and group them together, then output the average depth of those pixels.
        # This is necessary, because if the prompt is "floor", anything above the floor will all be considered part of the "negative mask"
        # and it will not be separated into separate objects.

        # Uses a DBSCAN (Density-Based Spatial Clustering of Applications with Noise) clustering method to group pixels with similar depth values
        depth_image = self.depth_image
        # Preprocess depth image (filter invalid depths)
        depth_image = depth_image.astype(np.float32)
        valid_mask = depth_image > 0  # Assuming depth values of 0 are invalid

        # Get the coordinates and depth values of valid pixels
        coords = np.column_stack(np.nonzero(valid_mask))
        depth_values = depth_image[coords[:, 0], coords[:, 1]]
        points = np.column_stack((coords, depth_values))

        # Apply DBSCAN clustering
        db = DBSCAN(eps=EPS, min_samples=MIN_SAMPLES, metric='euclidean').fit(points)
        labels = db.labels_

        # Create masks for each cluster
        num_labels = len(set(labels)) - (1 if -1 in labels else 0)
        masks = [np.zeros(depth_image.shape, dtype=bool) for _ in range(num_labels)]

        for label in range(num_labels):
            masks[label][coords[labels == label, 0], coords[labels == label, 1]] = True

        # Map masks to rgb image so that floor can be filtered out TODO

        # filter out any masks that are in the floor region

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

if __name__ == '__main__':
    try:
        obstacle_avoidance = ObstacleAvoidance()
        obstacle_avoidance.spin()
    except rospy.ROSInterruptException:
        pass