#!/usr/bin/env python3
# Authored by: Andor Siegers

import rospy
from object_detection.msg import MaskArray
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
from scipy.ndimage import label
from kinect_pub.srv import GetCameraInfo

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
        self.mask_image = None # RGB image accompanying mask data
        self.depth_image = None # Depth image accompanying mask data

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
        self.convert_mask_data(mask_data)

        # Use masks to calculate average distance of each unmasked section
        obstacles = self.find_obstacles() # 2D numbered list of obstacles ind(0) - index
                                                                        # ind(1) - average depth of obstacle

        # Use that information to calculate cost function
        # TODO


    def convert_mask_data(self, mask_data):
        try:
            # Empty arrays
            self.phrases = []
            self.centroids = []
            self.depth_vals = []
            self.logits = []
            self.masks = []         # boolean arrays
            self.mask_image = None  # RGB image accompanying mask data
            self.depth_image = None # Depth image accompanying mask data

            # Convert data back to accessible data types
            for temp_mask in mask_data.mask_data:
                temp_phrase = temp_mask.phrase
                centroid_loc = (int(temp_mask.centroid.x), int(temp_mask.centroid.y))  # Convert to integer
                depth_val = temp_mask.centroid.z / 1000
                temp_logit = temp_mask.logit

                mask_img = self.bridge.imgmsg_to_cv2(temp_mask.mask, desired_encoding="mono8")
                mask_img = (mask_img / 255).astype(bool)  # Convert back to boolean array
                
                self.phrases.append(temp_phrase)
                self.centroids.append(centroid_loc)
                self.depth_vals.append(depth_val)
                self.logits.append(temp_logit)
                self.masks.append(mask_img)

            # Convert rgb image to OpenCV format
            self.mask_image = self.bridge.imgmsg_to_cv2(mask_data.rgb_img, desired_encoding="bgr8")
            self.depth_image = self.bridge.imgmsg_to_cv2(mask_data.depth_img, desired_encoding="32FC1")

        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
        except Exception as e:
            rospy.logerr(f"Error processing mask data: {e}")
        
    def find_obstacles(self):
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
        obstacle_masks = split_mask(unmasked_area)

        # Find the average depth (or distance away from the camera) of each mask
        average_depths = calculate_average_depths(
            obstacle_masks, self.depth_image, self.rgb_intrinsics, self.depth_intrinsics, self.depth_dist_coeffs, self.extrinsic_matrix)
        
        return enumerate(average_depths)


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


if __name__ == '__main__':
    try:
        obstacle_avoidance = ObstacleAvoidance()
        obstacle_avoidance.spin()
    except rospy.ROSInterruptException:
        pass