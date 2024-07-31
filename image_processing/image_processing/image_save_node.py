#!/usr/bin/env python3
# Authored by: Andor Siegers

# -------------------------------------------- User defined constants ------------------------------------------------

# Directory to save images
SAVE_PATH = r'~/'

# Save folder name
SAVE_FOLDER_NAME = 'kinect_images'

# Frequency of images being saved
SAVE_RATE = 1 # in Hz

# Select whether to save normalized depth image (if false output will be black or white, not greyscale)
SAVE_NORM = False

# Select whether to save RGB images
SAVE_RGB = False

# Select whether to save depth images
SAVE_DEPTH = False

# Select whether to save IR images
SAVE_IR = False

# Select whether to save registered images
SAVE_REG = False

# Select whether to save masked images
# Note: since the processing time is much slower than other images, and the images get published on a different frequency than the raw images,
# this saves at the rate at which the processed images are published.
SAVE_MASK = True

# --------------------------------------------------------------------------------------------------------------------

# -------------------------------------------- Calculated constants --------------------------------------------------

# Calculate the save interval in seconds
SAVE_INTERVAL = 1.0 / SAVE_RATE

# --------------------------------------------------------------------------------------------------------------------

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import os
import shutil
import imageio

class ImageSaver(Node):
    def __init__(self):
        super().__init__('img_saver')
        self.bridge = CvBridge()

        # Clear the save directory
        self.clear_directory(SAVE_PATH)

        # Initialize indexes for file naming
        self.save_count_raw = 1
        self.save_count_mask = 1

        # Initialize images
        self.rgb_image = None
        self.depth_image = None
        self.ir_image = None
        self.reg_image = None
        self.mask_image = None

        # Define individual subscribers for each topic individually
        self.rgb_sub = self.create_subscription(Image, '/zed/zed_node/rgb/image_rect_color', self.callback_rgb, 10)
        self.depth_sub = self.create_subscription(Image, 'zed/zed_node/depth/depth_registered', self.callback_depth, 10)

        # Setup timer to call save function at specified rate
        self.save_timer = self.create_timer(SAVE_INTERVAL, self.save_images)

        # Call save function every time mask image is received
        self.mask_sub = self.create_subscription(Image, '/process/mask_img', self.save_mask_img, 10)
        
        self.get_logger().info("Image Saver Node Started")
    
    def clear_directory(self, path):
        # Remove all files in the save_folder_name directory (if it exists)
        if os.path.isdir(os.path.join(path, SAVE_FOLDER_NAME)):
            shutil.rmtree(os.path.join(path, SAVE_FOLDER_NAME))
            self.get_logger().info("Image Directory Cleared")

        # Create new folders to save to
        os.chdir(path)
        os.mkdir(SAVE_FOLDER_NAME)
        os.chdir(os.path.join(SAVE_PATH, SAVE_FOLDER_NAME))

        # Only makes folder if saving for corresponding image type is ON
        if SAVE_RGB:
            os.mkdir('rgb')
        if SAVE_DEPTH:
            os.mkdir('depth')
        if SAVE_MASK:
            os.mkdir('mask')
    
    def save_images(self):
        # Saves images to computer
        saved_img = False
        # RGB
        if SAVE_RGB and self.rgb_image is not None:
            os.chdir(os.path.join(SAVE_PATH, SAVE_FOLDER_NAME, "rgb"))
            name_str_rgb = "rgb_" + str(self.save_count_raw).zfill(6) + ".png"
            cv2.imwrite(str(name_str_rgb), self.rgb_image)
            saved_img = True

        # depth
        if SAVE_DEPTH and self.depth_image is not None:
            os.chdir(os.path.join(SAVE_PATH, SAVE_FOLDER_NAME, "depth"))
            name_str_depth = "depth_" + str(self.save_count_raw).zfill(6) + ".tiff"
            imageio.imwrite(str(name_str_depth), self.depth_image)
            saved_img = True
        
        if saved_img:
            self.save_count_raw += 1
            self.get_logger().info(f"Saved raw images at index {str(self.save_count_raw).zfill(6)}")

    def save_mask_img(self, mask_msg):
        try:
            self.mask_image = self.bridge.imgmsg_to_cv2(mask_msg, desired_encoding="rgb8")
            if SAVE_MASK and self.mask_image is not None:
                os.chdir(os.path.join(SAVE_PATH, SAVE_FOLDER_NAME, "mask"))
                name_str_mask = "mask_" + str(self.save_count_mask).zfill(6) + ".png"
                imageio.imwrite(str(name_str_mask), self.mask_image)
                self.save_count_mask += 1
                self.get_logger().info(f"Saved mask images at index {str(self.save_count_mask).zfill(6)}")
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert mask image: {e}")
        except Exception as e:
            self.get_logger().error(f"Error in save_mask_img: {e}")

    def callback_rgb(self, rgb_msg):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert rgb image: {e}")
        except Exception as e:
            self.get_logger().error(f"Error in callback_rgb: {e}")

    def callback_depth(self, depth_msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert depth image: {e}")
        except Exception as e:
            self.get_logger().error(f"Error in callback_depth: {e}")

def main(args=None):
    rclpy.init(args=args)
    image_saver = ImageSaver()
    rclpy.spin(image_saver)
    image_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
