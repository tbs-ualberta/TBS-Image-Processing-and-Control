#!/usr/bin/env python3
# Authored by: Andor Siegers

# Assumes that:
#  - the object_detection program is detecting "floor" as it's prompt
# Integrating the target into this program is doable, but the target would have to be detected and mapped as the goal instead of as an obstacle.

import rospy
from image_processing.msg import MaskArray
from cv_bridge import CvBridge
import numpy as np
import cv2
import math
from utils import unpack_MaskArray, unpack_RegistrationData, is_mask_overlapping, get_camera_parameters, display_depth_with_masks
from sklearn.cluster import DBSCAN
from geometry_msgs.msg import Twist

# ----------------------------------------------------- Constants -------------------------------------------------------
EPS = 10 # for DBSCAN - maximum distance between two depth samples for them to be considered as neighbors (in mm)

MIN_SAMPLES = 30 # for DBSCAN - the minimum number of points required to form an obstacle mask

OVERLAP_THRESHOLD = 0.1 # specifies how much of a mask must be contained in the floor mask to be filtered out

FLOOR_RECOGNITION_THRESHOLD = 0.3 # specifies how confident the model needs to be for the floor mask to be used
# -----------------------------------------------------------------------------------------------------------------------

class ObstacleAvoidance:
    class Obstacle:
        def __init__(self):
            self.mask_depth = None              # boolean mask of the obstacle in depth pixel space
            self.avg_depth = None               # average depth of the obstacle
            self.min_depth = None               # minimum depth of the obstacle
            self.max_depth = None               # maximum depth of the obstacle
            self.centroid_px = []               # location of the centroid of the obstacle in depth pixel space (x,y in px)
            self.centroid_cartesian = []        # location of the centroid in Cartesian space (x,y,z in mm)
            self.closest_point_px = []          # location of the closest point to the robot in depth pixel space (x,y in px)
            self.closest_point_cartesian = []   # location of the closest point in Cartesian space (x,y,z in mm)

    class Obstacle_2D:
        def __init__(self):
            self.closest_point = None   # coordinate of closest point from a top down view
            self.dist = None            # distance from the camera to the obstacle in the horizontal plane

    def __init__(self):
        self.bridge = CvBridge()

        # Initialize node
        rospy.init_node('obstacle_avoidance', anonymous=True)

        # Get camera parameters and store them in dictionaries
        self.depth_params, self.rgb_params = get_camera_parameters()
        if self.depth_params is None or self.rgb_params is None:
            rospy.logerr("Unable to get camera parameters.")
        else:
            rospy.loginfo("Fetched camera parameters successfully.")

        # Initialize array to store array of floor masks
        self.floor_masks = []   # 3D boolean array of floor masks (in depth pixel space)

        # Initialize boolean that indicates whether data is up to date
        self.data_updated = False

        # Initialize images and maps
        self.rgb_image = None           # RGB image accompanying mask data
        self.depth_image = None         # Depth image accompanying mask data
        self.undistorted_image = None   # Undistorted Depth image
        self.bigdepth_image = None      # Maps depth onto RGB
        self.colour_depth_map = None    # Maps RGB onto depth
        self.global_start_time = None   # Time at which the depth and RGB images were initially taken

        self.obstacles = []
        self.obstacles_2d = []  # stores a list of 2d obstacles
        
        '''
        To sync target and mask_data
        
        # Define subscribers for mask and target data
        self.mask_sub = message_filters.Subscriber('/process/mask_data', MaskArray)
        self.target_sub = message_filters.Subscriber('/process/target', Point)

        # Synchronize the topics (excluding mask)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.mask_sub, self.target_sub], 10, 1.0, allow_headerless=True)
        self.ts.registerCallback(self.calculate_path)
        '''

        # Define subscribers for mask and target data
        self.mask_sub = rospy.Subscriber('/process/mask_data', MaskArray, self.mask_data_callback)

        # Define velocity publisher
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        rospy.loginfo("Obstacle avoidance node started")

    def mask_data_callback(self, mask_data):
        # Convert raw mask data from message to usable datatypes
        mask_phrases, centroids, depth_vals, potential_floor_logits, potential_floor_masks, reg_data = unpack_MaskArray(mask_data)
        self.rgb_image, self.depth_image, self.undistorted_image, __, self.bigdepth_image, self.colour_depth_map, self.global_start_time = unpack_RegistrationData(reg_data)

        # filter out all masks that are not labelled as "floor" and not above floor recognition threshold
        for phrase, logit, potential_floor_mask in zip(mask_phrases, potential_floor_logits, potential_floor_masks):
            if "floor" in phrase and logit >= FLOOR_RECOGNITION_THRESHOLD:
                # Convert mask to uint8 type
                int_mask = potential_floor_mask.astype(np.uint8) * 255  # Convert True/False to 255/0
                # Resize floor mask to fit depth image
                int_mask_resized = cv2.resize(int_mask, (self.depth_image.shape[1], self.depth_image.shape[0]))
                # Convert mask back to boolean type
                bool_mask_resized = (int_mask_resized != 0)
                self.floor_masks.append(bool_mask_resized)

            # TODO save target information
        
        self.data_updated = True

    def calculate_path(self):
        rospy.loginfo("Calculating Path...")
        
        # Save start times for performance evaluation
        start_time = rospy.Time.now()
        global_start_time = self.global_start_time

        obstacles = [] # stores a list of Obstacle objects

        # Use masks to calculate average distance of each depth cluster
        obstacles, depth_image = self.find_obstacles()

        # Print times (for testing)
        dbscan_dif = rospy.Time.now() - start_time
        rospy.loginfo("DBSCAN time: %d.%09ds", dbscan_dif.secs, dbscan_dif.nsecs)
        total_dif = rospy.Time.now() - global_start_time
        rospy.loginfo("Total time taken from image capture: %d.%09ds", total_dif.secs, total_dif.nsecs)

        # Display masks (for testing)
        obstacle_masks = []
        for temp_obs in obstacles:
            obstacle_masks.append(temp_obs.mask_depth)
        display_depth_with_masks(depth_image, obstacle_masks)

        # Print total time (for testing)
        total_dif = rospy.Time.now() - global_start_time
        rospy.loginfo("Total time taken from image capture (after display): %d.%09ds", total_dif.secs, total_dif.nsecs)

        # Convert the 3D data to a top-down view
        # self.convert_to_top_down(obstacles)
        
        # Use that information to calculate cost function

        # TODO build a map of the environment and use potential field path planning to find a path

        # Display obstacle image (for testing)
        # self.display_obstacles()

        self.data_updated = False
        
    def find_obstacles(self):
        # Finds similar value depth pixels and groups them together using DBSCAN, then outputs the average depth of those pixels.
        # This is necessary, because if the prompt is "floor", anything above the floor will all be considered part of the "negative mask"
        # and it will not be separated into separate objects.

        # Reset obstacles array
        obstacles = []

        # TODO This runs very slowly: around 1 second per depth image calculation. This can be optimized by reducing the image quality and running
        # the same algorithm on the reduced image

        # Save variables needed so if they get updated during calculation, it doesn't affect the calculation
        rgb_image = self.rgb_image
        depth_image = self.depth_image
        undistorted_image = self.undistorted_image
        colour_depth_map = self.colour_depth_map
        floor_masks = self.floor_masks

        # Uses a DBSCAN (Density-Based Spatial Clustering of Applications with Noise) clustering method to group pixels with similar depth values
        # Flatten the image and create a list of points (x, y, depth)
        h, w = undistorted_image.shape
        points = []

        for i in range(h):
            for j in range(w):
                depth = undistorted_image[i, j]
                if depth > 0:  # Filter out zero depth points
                    points.append([i, j, depth])

        points = np.array(points)

        # Apply DBSCAN clustering
        db = DBSCAN(eps=EPS, min_samples=MIN_SAMPLES).fit(points)
        labels = db.labels_

        # Add labels to points
        points_with_labels = np.hstack((points, labels[:, np.newaxis]))

        # Initialize array for boolean masks
        cluster_masks = []
        unique_labels = set(labels)

        # Remove noise if present
        num_clusters = len(unique_labels) - (1 if -1 in unique_labels else 0)

        # Convert cluster information to boolean masks
        for cluster_id in range(num_clusters):
            # Create boolean mask for current cluster
            temp_mask = np.zeros((h,w), dtype=bool)

            # Get points in current cluster
            cluster_points = points_with_labels[labels == cluster_id]
            
            # Set all points in mask corresponding to current cluster to True
            for point in cluster_points:
                x, y, _, _ = point
                temp_mask[int(x), int(y)] = True

            cluster_masks.append(temp_mask)
        
        # Obtain and save obstacle data
        for cluster_mask in cluster_masks: # TODO fix this

            # Filter out any masks that overlap with the floor region
            if not is_mask_overlapping(cluster_mask, floor_masks, OVERLAP_THRESHOLD):
                temp_obstacle = self.Obstacle()

                # Save only cluster masks that are not overlapping
                temp_obstacle.mask_depth = cluster_mask

                # Calculate average, min, and max depth for the current cluster
                cluster_depth_values = undistorted_image[cluster_mask]
                temp_obstacle.avg_depth = np.mean(cluster_depth_values)
                temp_obstacle.min_depth = np.min(cluster_depth_values)
                temp_obstacle.max_depth = np.max(cluster_depth_values)

                # Calculate centroid for current cluster
                # temp_obstacle.centroid_px = calculate_centroids(cluster_mask)

                # Convert centroid location to Cartesian space
                # temp_obstacle.centroid_cartesian = get_point_xyz(undistorted_image, temp_obstacle.centroid_px, self.depth_params)

                # Calculate location of minimum value in array
                # temp_obstacle.closest_point_px = np.unravel_index(cluster_depth_values.argmin(), cluster_depth_values.shape)

                # Convert closest point location to Cartesian space
                # temp_obstacle.closest_point_cartesian = get_point_xyz(undistorted_image, temp_obstacle.closest_point_px, self.depth_params)

                # Add new obstacle to global list of obstacles
                obstacles.append(temp_obstacle)
                
                # TODO if this obstacle is considered the target, do not include in obstacle array

        return obstacles, depth_image

    def convert_to_top_down(self, obstacles):
        # Convert the obstacles (and target) locations to 2D top-down view

        '''
        The following ASCII art depicts the top down view of the robot and the location of the axes when calculating the cartesian
        coordinates of obstacles. Note that the origin is located at the front of the robot (where the depth sensor is placed).

        Top down view of Robot
             y-axis
                  |  /
         _________| /
        |         |/
        |   WMM   |--------------- x-axis
        |_________|\ <- 35.6deg
                    \
                     \

        '''
        # Calculate top down X and Y coords (assume positive x-axis goes from center of camera straight out in front
        # of the robot, with origin at camera sensor.
        # Note: the horizontal plane will be perpendicular to the sensor, not necessarily parallel with the floor.

        obstacles_2d = []

        # TODO adapt this to project all points in the cluster to 2D space and save them
        for temp_obstacle in obstacles:
            point_3d = temp_obstacle.closest_point_cartesian

            # Get x and y coordinates
            x = -1 * point_3d[2] # -1*z
            y = point_3d[0]      # x

            # Calculate hypotenuse (or distance to object in horizontal plane)
            h = math.sqrt(x * x + y * y)

            temp_2d_obstacle = self.Obstacle_2D()
            temp_2d_obstacle.closest_point = [x, y]
            temp_2d_obstacle.dist = h
            obstacles_2d.append(temp_2d_obstacle)

        # TODO do this for target as well

        return obstacles_2d
        

    def display_obstacles_with_data(self):
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
                temp_centroid = temp_obstacle.centroid_px
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
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            if obstacle_avoidance.data_updated:
                obstacle_avoidance.calculate_path()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass