#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import message_filters
import cv2
from cv_bridge import CvBridge, CvBridgeError
import os
import shutil
import numpy as np
import open3d as o3d
import imageio
# import matplotlib.pyplot as plt
# import requests
# from PIL import Image
# from io import BytesIO
# from lang_sam import LangSAM
# import time

# -------------------------------------------- User defined constants ------------------------------------------------

# Directory to save images
SAVE_PATH = r'/home/wmm'

# Save folder name
SAVE_FOLDER_NAME = 'kinect_images'

# Selects whether files should process or not
PROCESS_IMAGES = True

# Processing frequency
PROCESSING_RATE = 0.5 # in Hz

# Select whether images should save or not
SAVE_IMAGES = False

# Frequency of images being saved
SAVE_RATE = 1 # in Hz

# --------------------------------------------------------------------------------------------------------------------

# --------------------------------------- Camera calibration constants -----------------------------------------------
# Intrinsic parameters
DEPTH_INTRINSICS = np.array([[366.193, 0, 256.684],
                             [0, 366.193, 207.085],
                             [0, 0, 1]])
DEPTH_DIST_COEFFS = np.array([0.0893804, -0.272566, 0, 0, 0.0958438])

RGB_INTRINSICS = np.array([[1081.37, 0, 959.5],
                           [0, 1081.37, 539.5],
                           [0, 0, 1]])

R = np.eye(3)  # Rotation matrix (3x3)
T = np.array([0, 0, 0])  # Translation vector (3x1)

# --------------------------------------------------------------------------------------------------------------------

# -------------------------------------------- Calculated constants --------------------------------------------------

# Calculate the process interval in seconds
PROCESSING_INTERVAL = 1.0 / PROCESSING_RATE

# Calculate the save interval in seconds
SAVE_INTERVAL = 1.0 / SAVE_RATE

EXTRINSIC_MATRIX = np.eye(4)
EXTRINSIC_MATRIX[:3, :3] = R
EXTRINSIC_MATRIX[:3, 3] = T

# --------------------------------------------------------------------------------------------------------------------

class ImageSaverProcessor:
    def __init__(self):
        self.bridge = CvBridge()

        self.pcd = o3d.geometry.PointCloud()

        # Clear the save directory
        if SAVE_IMAGES:
            self.clear_directory(SAVE_PATH)

        # Initialize node
        rospy.init_node('image_saver_processor', anonymous=True)

        # for file saving
        self.last_save_time = rospy.Time.now()
        self.save_count = 1

        # Define subscribers for depth and rgb topics
        self.rgb_sub = message_filters.Subscriber('/kinect2/rgb', Image)
        self.depth_sub = message_filters.Subscriber('/kinect2/depth_raw', Image)

        # Synchronize the topics
        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], 10, 1.0)
        self.ts.registerCallback(self.callback)

        # Initialize images
        self.rgb_image = None
        self.depth_image = None

        # Set up a timers to call save and process functions
        if PROCESS_IMAGES:
            self.processing_timer = rospy.Timer(rospy.Duration(PROCESSING_INTERVAL), self.process_images)
        if SAVE_IMAGES:
            self.save_timer = rospy.Timer(rospy.Duration(SAVE_INTERVAL), self.save_images)

        rospy.loginfo("Image Processor Node Started")

    def callback(self, rgb_msg, depth_msg):
        try:
            # Convert the rgb and depth images to OpenCV format
            self.rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            self.depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="32FC1")
        except CvBridgeError as e:
            rospy.logerr(f"Failed to convert images: {e}")

    def clear_directory(self, path):
        # Remove all files in the save_folder_name directory (if it exists)
        if os.path.isdir(os.path.join(path, SAVE_FOLDER_NAME)):
            shutil.rmtree(os.path.join(path, SAVE_FOLDER_NAME))
            rospy.loginfo("Image Directory Cleared")

        # create new folders to save to
        os.chdir(path)
        os.mkdir(SAVE_FOLDER_NAME)
        os.chdir(os.path.join(SAVE_PATH, SAVE_FOLDER_NAME))
        # only makes folder if saving for corresponding image type is ON
        os.mkdir('rgb')
        os.mkdir('depth')
    
    def save_images(self, event):
        # Saves images to computer
        # TODO this skips images a lot, so it should be changed, but it works for now
        if self.rgb_image is not None and self.depth_image is not None:
            # rgb
            os.chdir(os.path.join(SAVE_PATH, SAVE_FOLDER_NAME, "rgb"))
            path_str_rgb = "rgb_" + str(self.save_count).zfill(6) + ".png"
            cv2.imwrite(str(path_str_rgb), self.rgb_image)

            # depth
            os.chdir(os.path.join(SAVE_PATH, SAVE_FOLDER_NAME, "depth"))
            path_str_depth = "depth_" + str(self.save_count).zfill(6) + ".tiff"
            imageio.imwrite(str(path_str_depth), self.depth_image)

            self.save_count += 1

            rospy.loginfo(f"Saved images at index" + str(self.save_count).zfill(6))

    def map_rgb_to_depth(rgb_x, rgb_y, depth_image):
        """Map an RGB pixel to the corresponding depth pixel and retrieve the depth value."""
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
            raise ValueError("The computed depth pixel is out of bounds.")

    def process_images(self, event):
        if self.rgb_image is not None and self.depth_image is not None:
            rospy.loginfo("Processing images")

            # -----------------------------Process Images----------------------------------

            rgb_x = 5
            rgb_y = 100
            
            # Calculate corresponding depth value of from RGB pixel coordinates
            depth_val = self.map_rgb_to_depth(rgb_x, rgb_y, self.depth_image)

            rospy.loginfo("The depth value for the point (" + str(rgb_x) + ", " + str(rgb_y) + ") is " + str(depth_val) + "mm.")

            # -----------------------------------------------------------------------------

            # Reset images after processing
            self.rgb_image = None
            self.depth_image = None

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        image_saver_processor = ImageSaverProcessor()
        image_saver_processor.spin()
    except rospy.ROSInterruptException:
        pass

# TODO add image processing to find objects in frame