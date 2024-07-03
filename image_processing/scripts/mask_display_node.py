#!/usr/bin/env python3
# Authored by: Andor Siegers

import rospy
from image_processing.msg import MaskArray
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from process_helper import unpack_MaskArray, unpack_RegistrationData

class MaskDisplay:
    def __init__(self):
        self.bridge = CvBridge()

        # Initialize node
        rospy.init_node('mask_display', anonymous=True)

        # Initialize mask data arrays
        self.phrases = []
        self.centroids = []
        self.depth_vals = []
        self.logits = []
        self.masks = [] # boolean array
        self.rgb_image = None # RGB image accompanying mask data
        self.depth_image = None # Depth image accompanying mask data

        # Define subscriber for masks
        self.mask_sub = rospy.Subscriber('/process/mask_data', MaskArray, self.mask_callback)

        # Define publisher to publish mask image
        self.mask_img_pub = rospy.Publisher('/process/mask_img', Image, queue_size=10)

        rospy.loginfo("Mask Display Node Started")

    def mask_callback(self, mask_data):
        # Runs whenever new mask data is received

        reg_data = None
        # Convert raw mask data from message to usable datatypes
        self.phrases, self.centroids, self.depth_vals, self.logits, self.masks, reg_data = unpack_MaskArray(mask_data)
        self.rgb_image, self.depth_image, __, __, __, __, __ = unpack_RegistrationData(reg_data)

        # Display the mask data overlayed over rgb image
        self.display_masks()
    
    def display_masks(self):
        try:
            if self.rgb_image is None:
                rospy.logwarn("rgb_image is None, skipping display")
                return
            
            overlay = self.rgb_image.copy()
            alpha = 0.5  # Transparency factor
            
            mask_sum = np.zeros_like(self.rgb_image)

            # Convert mask to single array. This allows for even application of colour and the image is not too darkened by multiple masks being overlaid.
            for mask, logit in zip(self.masks, self.logits):
                mask_scaled = np.zeros_like(self.rgb_image)
                # Scale proportionally to confidence value
                scaled_val = [0, 255 * logit, 0] # RGB values that determine mask colour
                mask_scaled[mask] = scaled_val
                # Combine arrays
                mask_sum = np.maximum(mask_sum, mask_scaled)

            # Overlay mask on the image
            cv2.addWeighted(mask_sum, alpha, overlay, 1 - alpha, 0, overlay)

            for centroid, phrase, depth, logit in zip(self.centroids, self.phrases, self.depth_vals, self.logits):
                # Draw centroid
                cv2.circle(overlay, (centroid[0], centroid[1]), 5, (0, 0, 255), -1)  # Red colour for centroid

                # Put text for phrase and depth value
                cv2.putText(overlay, f"{phrase}, {depth:.2f}m", (centroid[0] + 10, centroid[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

            # Publish image to ROS topic for image saving
            self.mask_img_pub.publish(self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8"))

            # Display image in window
            cv2.imshow("Mask Overlay Image", overlay)

            # Uncomment to check if mask is displaying properly - for testing
            # cv2.imshow("Depth Image", self.depth_image)

            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f"Error displaying masks: {e}")
    
    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        image_saver_processor = MaskDisplay()
        image_saver_processor.spin()
    except rospy.ROSInterruptException:
        pass
