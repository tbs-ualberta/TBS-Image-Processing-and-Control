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
from process_helper import unpack_MaskArray, unpack_RegistrationData, map_depth_mask_to_rgb, is_mask_overlapping, calculate_centroids
from sklearn.cluster import DBSCAN

# ----------------------------------------------------- Constants -------------------------------------------------------
EPS = 0.05 # for DBSCAN - maximum distance between two points for them to be considered as neighbors (in mm)

MIN_SAMPLES = 10 # for DBSCAN - the minimum number of points required to form a mask

OVERLAP_THRESHOLD = 0.5 # specifies how much of a mask must be contained in the floor mask to be filtered out
# -----------------------------------------------------------------------------------------------------------------------

class ObstacleAvoidance:
    class Obstacle:
        def __init__(self):
            self.mask_depth = None  # boolean mask of the obstacle in depth space
            self.mask_rgb = None    # boolean mask of the obstacle in RGB space
            self.avg_depth = None   # average depth of the obstacle
            self.min_depth = None   # minimum depth of the obstacle
            self.max_depth = None   # maximum depth of the obstacle
            self.centroid = []      # location of the centroid of the obstacle in RGB space (x,y)

    def __init__(self):
        self.bridge = CvBridge()

        # Initialize node
        rospy.init_node('obstacle_avoidance', anonymous=True)

        # Initialize mask data arrays
        self.phrases = []
        self.centroids = []
        self.depth_vals = []
        self.logits = []
        self.floor_masks = [] # boolean array
        self.rgb_image = None # RGB image accompanying mask data
        self.depth_image = None # Depth image accompanying mask data
        self.bigdepth_image = None # Maps depth onto RGB
        self.colour_depth_map = None # Maps RGB onto depth

        self.obstacles = [] # stores a list of Obstacle objects
        
        # Define subscriber for mask data
        self.mask_sub = rospy.Subscriber('/process/mask_data', MaskArray, self.mask_callback)

        # Define publishers

        rospy.loginfo("Obstacle avoidance node started")

    def mask_callback(self, mask_data):
        # Runs whenever new mask data is received

        # Convert raw mask data from message to usable datatypes
        self.phrases, self.centroids, self.depth_vals, self.logits, self.floor_masks, reg_data = unpack_MaskArray(mask_data)
        self.rgb_image, self.depth_image, __, self.bigdepth_image, self.colour_depth_map = unpack_RegistrationData(reg_data)

        # Use masks to calculate average distance of each depth cluster
        self.find_obstacles()

        # Display obstacle image (for testing)
        self.display_obstacles()

        # Use that information to calculate cost function
        # TODO build a map of the environment and use potential field path planning to find a path
        
    def find_obstacles(self):
        # Finds similar value depth pixels and groups them together, then outputs the average depth of those pixels.
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
        cluster_masks = [np.zeros(depth_image.shape, dtype=bool) for _ in range(num_labels)]

        for label in range(num_labels):
            cluster_masks[label][coords[labels == label, 0], coords[labels == label, 1]] = True
        
        self.obstacles = []
        for mask in cluster_masks:
            # Map mask to RGB space
            rgb_mask = map_depth_mask_to_rgb(mask, self.rgb_image, self.depth_image, self.colour_depth_map)

            # Filter out any masks that overlap with the floor region
            if not is_mask_overlapping(rgb_mask, self.floor_masks, OVERLAP_THRESHOLD):

                obstacle = self.Obstacle()
                # save only cluster masks that are not overlapping
                obstacle.mask_depth = mask
                obstacle.mask_rgb = rgb_mask

                # Calculate average, min, and max depth for the current cluster
                cluster_depth_values = depth_image[mask]
                obstacle.avg_depth = np.mean(cluster_depth_values)
                obstacle.min_depth = np.min(cluster_depth_values)
                obstacle.max_depth = np.max(cluster_depth_values)

                # Calculate centroid for current cluster
                obstacle.centroid = calculate_centroids(rgb_mask)

                # Add new obstacle to global list of obstacles
                self.obstacles.append(obstacle)


    def display_obstacles(self):
        # TODO check this
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