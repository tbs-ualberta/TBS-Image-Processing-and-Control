#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import message_filters
import cv2
from cv_bridge import CvBridge, CvBridgeError

# -------------------------------------------- User defined constants ------------------------------------------------

# Select whether to displayed normalized image map
DISPLAY_NORM = True

# Frequency of images being displayed
DISPLAY_RATE = 20 # in Hz

# --------------------------------------------------------------------------------------------------------------------

# -------------------------------------------- Calculated constants --------------------------------------------------

# Calculate the display interval in seconds
DISPLAY_INTERVAL = 1.0 / DISPLAY_RATE

# --------------------------------------------------------------------------------------------------------------------

class ImageSaverProcessor:
    def __init__(self):
        self.bridge = CvBridge()

        # Initialize node
        rospy.init_node('img_display', anonymous=True)

        # Define subscribers for depth and rgb topics
        self.rgb_sub = message_filters.Subscriber('/kinect2/rgb', Image)
        self.reg_sub = message_filters.Subscriber('/kinect2/reg', Image)

        # Choose which topic to subscribe to, depending on whether normalization should be displayed
        if DISPLAY_NORM:
            self.depth_sub = message_filters.Subscriber('/kinect2/depth_norm', Image)
        else:
            self.depth_sub = message_filters.Subscriber('/kinect2/depth_raw', Image)

        # Synchronize the topics   
        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub, self.reg_sub], 10, 1.0)
        self.ts.registerCallback(self.callback)

        # Initialize images
        self.rgb_image = None
        self.depth_image = None
        self.reg_image = None

        self.display_timer = rospy.Timer(rospy.Duration(DISPLAY_INTERVAL), self.display_images)

        rospy.loginfo("Image Display Node Started")
        
    def callback(self, rgb_msg, depth_msg, reg_msg):
        try:
            # Convert the rgb and depth images to OpenCV format
            self.rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            self.depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="32FC1")
            self.reg_image = self.bridge.imgmsg_to_cv2(reg_msg, desired_encoding="passthrough")
        except CvBridgeError as e:
            rospy.logerr(f"Failed to convert images: {e}")

    def display_images(self, event):
        # Display the images using OpenCV
        if self.rgb_image is not None and self.rgb_image.size > 0:
            cv2.imshow("RGB Image", self.rgb_image)
        if self.depth_image is not None and self.depth_image.size > 0:
            cv2.imshow("Depth Image", self.depth_image)
        if self.reg_image is not None and self.reg_image.size > 0:
            cv2.imshow("Registered Image", self.rgb_image)
        cv2.waitKey(1)

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        image_saver_processor = ImageSaverProcessor()
        image_saver_processor.spin()
    except rospy.ROSInterruptException:
        pass


# TODO figure out why the registered image is the same as the rgb image
# TODO add visualization for the masking algorithm, centroid locations, and depth values