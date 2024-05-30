#!/usr/bin/env python3
# Authored by: Andor Siegers

# -------------------------------------------- User defined constants ------------------------------------------------

# Directory to save images
SAVE_PATH = r'/home/wmm'

# Save folder name
SAVE_FOLDER_NAME = 'kinect_images'

# Frequency of images being saved
SAVE_RATE = 1 # in Hz

# Select whether to save normalized depth image (if false output will be black or white, not greyscale)
SAVE_NORM = True

# Select whether to save RGB images
SAVE_RGB = True

# Select whether to save RGB images
SAVE_DEPTH = True

# Select whether to save RGB images
SAVE_IR = False

# Select whether to save RGB images
SAVE_REG = False

# --------------------------------------------------------------------------------------------------------------------

# -------------------------------------------- Calculated constants --------------------------------------------------

# Calculate the save interval in seconds
SAVE_INTERVAL = 1.0 / SAVE_RATE

# --------------------------------------------------------------------------------------------------------------------

import rospy
from sensor_msgs.msg import Image
import message_filters
import cv2
from cv_bridge import CvBridge, CvBridgeError
import os
import shutil
import imageio

class ImageSaver:
    def __init__(self):
        self.bridge = CvBridge()

        # Initialize node
        rospy.init_node('img_display', anonymous=True)

        # Define subscribers for depth and rgb topics
        self.rgb_sub = message_filters.Subscriber('/kinect2/rgb', Image)
        self.ir_sub = message_filters.Subscriber('/kinect2/ir_norm', Image)
        self.reg_sub = message_filters.Subscriber('/kinect2/reg', Image)

        # Choose which topic to subscribe to, depending on whether normalization should be displayed
        if SAVE_NORM:
            self.depth_sub = message_filters.Subscriber('/kinect2/depth_norm', Image)
        else:
            self.depth_sub = message_filters.Subscriber('/kinect2/depth_raw', Image)

        # Clear the save directory
        self.clear_directory(SAVE_PATH)

        self.last_save_time = rospy.Time.now()
        self.save_count = 1

        # Initialize images
        self.rgb_image = None
        self.depth_image = None
        self.ir_image = None
        self.reg_image = None

        # Synchronize the topics
        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub, self.ir_sub, self.reg_sub], 10, 1.0, allow_headerless=True)
        self.ts.registerCallback(self.callback)

        # Setup timer to call save function at specified rate
        self.save_timer = rospy.Timer(rospy.Duration(SAVE_INTERVAL), self.save_images)
        
        rospy.loginfo("Image Saver Node Started")

    def callback(self, rgb_msg, depth_msg, ir_msg, reg_msg):
        try:
            # Convert the rgb and depth images to OpenCV format
            self.rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            self.depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="32FC1")
            self.ir_image = self.bridge.imgmsg_to_cv2(ir_msg, desired_encoding="32FC1")
            self.reg_image = self.bridge.imgmsg_to_cv2(reg_msg, desired_encoding="bgr8")

        except CvBridgeError as e:
            rospy.logerr(f"Failed to convert images: {e}")
        except Exception as e:
            rospy.logerr(f"Error in callback_no_mask: {e}")
    
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
        if SAVE_RGB:
            os.mkdir('rgb')
        if SAVE_DEPTH:
            os.mkdir('depth')
        if SAVE_IR:
            os.mkdir('ir')
        if SAVE_REG:
            os.mkdir('reg')
    
    def save_images(self, event):
        # Saves images to computer
        # RGB
        if SAVE_RGB and self.rgb_image is not None:
            os.chdir(os.path.join(SAVE_PATH, SAVE_FOLDER_NAME, "rgb"))
            path_str_rgb = "rgb_" + str(self.save_count).zfill(6) + ".png"
            cv2.imwrite(str(path_str_rgb), self.rgb_image)

        # depth
        if SAVE_DEPTH and self.depth_image is not None:
            os.chdir(os.path.join(SAVE_PATH, SAVE_FOLDER_NAME, "depth"))
            path_str_depth = "depth_" + str(self.save_count).zfill(6) + ".tiff"
            imageio.imwrite(str(path_str_depth), self.depth_image)

        # IR
        if SAVE_IR and self.ir_image is not None:
            os.chdir(os.path.join(SAVE_PATH, SAVE_FOLDER_NAME, "ir"))
            path_str_ir = "ir_" + str(self.save_count).zfill(6) + ".tiff"
            imageio.imwrite(str(path_str_ir), self.ir_image)
        
        # reg
        if SAVE_REG and self.reg_image is not None:
            os.chdir(os.path.join(SAVE_PATH, SAVE_FOLDER_NAME, "reg"))
            path_str_reg = "reg_" + str(self.save_count).zfill(6) + ".png"
            imageio.imwrite(str(path_str_reg), self.reg_image)

        self.save_count += 1

        rospy.loginfo(f"Saved images at index" + str(self.save_count).zfill(6))

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        image_saver = ImageSaver()
        image_saver.spin()
    except rospy.ROSInterruptException:
        pass