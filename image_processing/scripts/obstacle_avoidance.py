#!/usr/bin/env python3
# Authored by: Andor Siegers

# Assumes that the object_detection program is detecting "floor" as it's prompt
# Integrating the target into this program is doable, but the target would have to be detected and mapped as the goal instead of an obstacle.

import rospy
from image_processing.msg import MaskArray
from cv_bridge import CvBridge
import numpy as np
import cv2
from process_helper import unpack_MaskArray, unpack_RegistrationData, map_depth_mask_to_rgb, is_mask_overlapping, calculate_centroids
from sklearn.cluster import DBSCAN
from geometry_msgs.msg import Point, Twist
import message_filters

# ----------------------------------------------------- Constants -------------------------------------------------------
EPS = 5 # for DBSCAN - maximum distance between two depth samples for them to be considered as neighbors (in mm) TODO validate that this is in mm

MIN_SAMPLES = 10 # for DBSCAN - the minimum number of points required to form an obstacle mask

OVERLAP_THRESHOLD = 0.5 # specifies how much of a mask must be contained in the floor mask to be filtered out

FLOOR_RECOGNITION_THRESHOLD = 0.3 # specifies how confident the model needs to be for the floor mask to be saved
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
            self.closest_point = [] # location of the closest point to the robot in RGB space (x,y)

    def __init__(self):
        self.bridge = CvBridge()

        # Initialize node
        rospy.init_node('obstacle_avoidance', anonymous=True)

        # Initialize array to store array of floor masks
        self.floor_masks = []   # 3D boolean array of floor masks

        # Initialize images and maps
        self.rgb_image = None # RGB image accompanying mask data
        self.depth_image = None # Depth image accompanying mask data
        self.bigdepth_image = None # Maps depth onto RGB
        self.colour_depth_map = None # Maps RGB onto depth

        self.obstacles = [] # stores a list of Obstacle objects
        
        # Define subscribers for mask and target data
        self.mask_sub = message_filters.Subscriber('/process/mask_data', MaskArray)
        self.target_sub = message_filters.Subscriber('/process/target', Point)

        # Synchronize the topics (excluding mask)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.mask_sub, self.target_sub], 10, 1.0, allow_headerless=True)
        self.ts.registerCallback(self.calculate_path)

        # Define velocity publisher
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        rospy.loginfo("Obstacle avoidance node started")

    def calculate_path(self, mask_data, target_location):
        # Runs whenever new mask data is received

        # Convert raw mask data from message to usable datatypes
        mask_phrases, centroids, depth_vals, potential_floor_logits, potential_floor_masks, reg_data = unpack_MaskArray(mask_data)
        self.rgb_image, self.depth_image, __, self.bigdepth_image, self.colour_depth_map = unpack_RegistrationData(reg_data)

        # filter out all masks that are not labelled as "floor" and not above floor recognition threshold
        for phrase, logit, potential_floor_mask in zip(mask_phrases, potential_floor_logits, potential_floor_masks):
            if "floor" in phrase and logit >= FLOOR_RECOGNITION_THRESHOLD:
                self.floor_masks.append(potential_floor_mask)
                
        # Use masks to calculate average distance of each depth cluster
        self.find_obstacles()

        # Display obstacle image (for testing)
        self.display_obstacles()

        # Convert the 3D data to a top-down view
        self.convert_to_top_down()
        
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
        for cluster_mask in cluster_masks:
            # Obtain and save obstacle data

            # Map cluster mask to RGB space
            rgb_mask = map_depth_mask_to_rgb(cluster_mask, self.rgb_image, self.depth_image, self.colour_depth_map)

            # Filter out any masks that overlap with the floor region
            if not is_mask_overlapping(rgb_mask, self.floor_masks, OVERLAP_THRESHOLD):

                temp_obstacle = self.Obstacle()
                # save only cluster masks that are not overlapping
                temp_obstacle.mask_depth = cluster_mask
                temp_obstacle.mask_rgb = rgb_mask

                # Calculate average, min, and max depth for the current cluster
                cluster_depth_values = depth_image[cluster_mask]
                temp_obstacle.avg_depth = np.mean(cluster_depth_values)
                temp_obstacle.min_depth = np.min(cluster_depth_values)
                temp_obstacle.max_depth = np.max(cluster_depth_values)

                # Calculate centroid for current cluster
                temp_obstacle.centroid = calculate_centroids(rgb_mask)

                # Calculate location of minimum value in array
                temp_obstacle.closest_point = np.unravel_index(cluster_depth_values.argmin(), cluster_depth_values.shape)

                # Add new obstacle to global list of obstacles
                self.obstacles.append(temp_obstacle)

    def convert_to_top_down(self):
        # Convert the obstacle (and target) locations to 2D top-down view

        # Convert from px to mm

        # Discard height information

        # Calculate top down X and Y coords (assume positive x-axis goes from center of camera straight out in front of the robot, 
        # with origin at camera sensor
        


    def display_obstacles(self):
        # TODO test this
        try:
            if self.rgb_image is None:
                rospy.logwarn("rgb_image is None, skipping display")
                return
            
            overlay = self.rgb_image.copy()
            alpha = 0.5  # Transparency factor
            mask_colour = [0, 255, 0] # RGB values that determine mask colour
            
            obstacle_mask_sum = np.zeros_like(self.rgb_image)

            # Convert obstacle masks to single array
            for temp_obstacle in self.obstacles:
                obstacle_mask_coloured = np.zeros_like(self.rgb_image)
                obstacle_mask_coloured[temp_obstacle.mask_rgb] = mask_colour
                # Combine arrays
                obstacle_mask_sum = np.maximum(obstacle_mask_sum, obstacle_mask_coloured)

            # Overlay mask on the image
            cv2.addWeighted(obstacle_mask_sum, alpha, overlay, 1 - alpha, 0, overlay)

            # Draw centroid and average depth
            for temp_obstacle in self.obstacles:
                temp_centroid = temp_obstacle.centroid
                # Draw centroid
                cv2.circle(overlay, (temp_centroid[0], temp_centroid[1]), 5, (0, 0, 255), -1)  # Red color for centroid

                # Put text for average depth
                cv2.putText(overlay, str(temp_obstacle.avg_depth), (temp_centroid[0] + 10, temp_centroid[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

            # Display image in window
            cv2.imshow("Obstacle Masks", overlay)

            # Uncomment to display corresponding depth image
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