#!/usr/bin/env python3
# Authored by: Andor Siegers

# -------------------------------------------- User defined constants ------------------------------------------------

# Select whether to display RGB image
DISPLAY_RGB = True

# Select whether to display depth image
DISPLAY_DEPTH = True

# Select whether to display infrared image
DISPLAY_IR = False

# Select whether to display normalized depth image (if false output will be black or white, not greyscale)
DISPLAY_NORM = True

# Select whether to display registered image
DISPLAY_REG = False

# Select whether to display images from the USB webcam
DISPLAY_USB = False

# Frequency of images being displayed
DISPLAY_RATE = 20 # in Hz

# --------------------------------------------------------------------------------------------------------------------

# -------------------------------------------- Calculated constants --------------------------------------------------

# Calculate the display interval in seconds
DISPLAY_INTERVAL = 1.0 / DISPLAY_RATE

# --------------------------------------------------------------------------------------------------------------------

import rospy
from sensor_msgs.msg import Image
import message_filters
import cv2
from cv_bridge import CvBridge, CvBridgeError

class ImageDisplay:
    def __init__(self):
        self.bridge = CvBridge()

        # Initialize node
        rospy.init_node('img_display', anonymous=True)

        # Define subscribers for depth and rgb topics
        self.rgb_sub = message_filters.Subscriber('/rgbd_out/rgb', Image)
        self.ir_sub = message_filters.Subscriber('/rgbd_out/ir_norm', Image)
        self.reg_sub = message_filters.Subscriber('/rgbd_out/reg', Image)

        # Choose which topic to subscribe to, depending on whether normalization should be displayed
        if DISPLAY_NORM:
            self.depth_sub = message_filters.Subscriber('/rgbd_out/depth_norm', Image)
        else:
            self.depth_sub = message_filters.Subscriber('/rgbd_out/depth_raw', Image)

        # Initialize images
        self.rgb_image = None
        self.depth_image = None
        self.ir_image = None
        self.reg_image = None
        self.usb_image = None

        # Subscriber to update usb image everytime one is received
        self.usb_cam_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.callback_usb)

        # Synchronize the topics
        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub, self.ir_sub, self.reg_sub], 10, 1.0, allow_headerless=True)
        self.ts.registerCallback(self.callback)

        # Timer to display all images
        self.display_timer = rospy.Timer(rospy.Duration(DISPLAY_INTERVAL), self.display_kinect_images)

        rospy.loginfo("Image Display Node Started")

    def display_kinect_images(self, event):
        try:
            # Display the images using OpenCV
            if DISPLAY_RGB and self.rgb_image is not None and self.rgb_image.size > 0:
                cv2.imshow("Kinect RGB Image", self.rgb_image)
            if DISPLAY_DEPTH and self.depth_image is not None and self.depth_image.size > 0:
                cv2.imshow("Kinect Depth Image", self.depth_image)
            if DISPLAY_IR and self.ir_image is not None and self.ir_image.size > 0:
                cv2.imshow("Kinect IR Image", self.ir_image)
            if DISPLAY_REG and self.reg_image is not None and self.reg_image.size > 0:
                cv2.imshow("Kinect Registered Image", self.reg_image)
            if self.usb_image is not None and self.usb_image.size > 0:
                cv2.imshow("Webcam Image", self.usb_image)
            
            cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(f"Error displaying images: {e}")
        
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

    def callback_usb(self, usb_img_msg):
        try:
            # Convert the usb rgb image to OpenCV format
            self.usb_image = self.bridge.imgmsg_to_cv2(usb_img_msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"Failed to convert images: {e}")
        except Exception as e:
            rospy.logerr(f"Error in callback_no_mask: {e}")

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        image_display = ImageDisplay()
        image_display.spin()
    except rospy.ROSInterruptException:
        pass