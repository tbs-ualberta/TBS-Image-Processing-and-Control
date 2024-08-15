#!/usr/bin/env python3
# Authored by: Andor Siegers

# -------------------------------------------- User defined constants ------------------------------------------------

# Select whether to display RGB image
DISPLAY_RGB = False

# Select whether to display depth image
DISPLAY_DEPTH = True

# Frequency of images being displayed
DISPLAY_RATE = 20 # in Hz

# --------------------------------------------------------------------------------------------------------------------

# -------------------------------------------- Calculated constants --------------------------------------------------

# Calculate the display interval in seconds
DISPLAY_INTERVAL = 1.0 / DISPLAY_RATE

# --------------------------------------------------------------------------------------------------------------------

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

class ImageDisplay(Node):
    def __init__(self):
        super().__init__('img_display')
        self.bridge = CvBridge()

        # Define individual subscribers for each topic individually
        self.rgb_sub = self.create_subscription(Image, 'zed/zed_node/rgb/image_rect_color', self.callback_rgb, 10)
        self.depth_sub = self.create_subscription(Image, 'zed/zed_node/depth/depth_registered', self.callback_depth, 10)

        # Initialize images
        self.rgb_image = None
        self.depth_image = None

        # Timer to display all images
        self.display_timer = self.create_timer(DISPLAY_INTERVAL, self.display_rgbd_images)

        self.get_logger().info("Image Display Node Started")

    def display_rgbd_images(self):
        try:
            # Display the images using OpenCV
            if DISPLAY_RGB and self.rgb_image is not None and self.rgb_image.size > 0:
                cv2.imshow("RGB Image", self.rgb_image)
            if DISPLAY_DEPTH and self.depth_image is not None and self.depth_image.size > 0:
                cv2.imshow("Depth Image", self.depth_image)
            
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error displaying images: {e}")

    def callback_rgb(self, rgb_msg):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgra8")
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert rgb image: {e}")
        except Exception as e:
            self.get_logger().error(f"Error in callback_rgb: {e}")

    def callback_depth(self, depth_msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="32FC1")
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert depth image: {e}")
        except Exception as e:
            self.get_logger().error(f"Error in callback_depth: {e}")

def main(args=None):
    rclpy.init(args=args)
    image_display = ImageDisplay()
    rclpy.spin(image_display)
    image_display.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
