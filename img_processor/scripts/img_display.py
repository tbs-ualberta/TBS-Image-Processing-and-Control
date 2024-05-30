#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from img_processor.msg import MaskArray, MaskData
import message_filters
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import threading  # Import threading module

# -------------------------------------------- User defined constants ------------------------------------------------

# Select whether to display RGB image
DISPLAY_RGB = True

# Select whether to display depth image
DISPLAY_DEPTH = True

# Select whether to displayed normalized depth image (if false output will be black or white, not greyscale)
DISPLAY_NORM = True

# Select whether to display registered image
DISPLAY_REG = True

# Select whether to display masks overlayed on rgb image
DISPLAY_MASKS = True

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

        # Define subscriber for masks
        self.mask_sub = rospy.Subscriber('/process/mask_data', MaskArray, self.process_mask_data)

        # Choose which topic to subscribe to, depending on whether normalization should be displayed
        if DISPLAY_NORM:
            self.depth_sub = message_filters.Subscriber('/kinect2/depth_norm', Image)
        else:
            self.depth_sub = message_filters.Subscriber('/kinect2/depth_raw', Image)

        # Initialize images
        self.rgb_image = None
        self.depth_image = None
        self.reg_image = None
        
        # Initialize mask data arrays
        self.phrases = []
        self.centroids = []
        self.depth_vals = []
        self.logits = []
        self.masks = [] # boolean array
        self.mask_image = None # RGB image accompanying mask data

        # Synchronize the topics
        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub, self.reg_sub], 10, 1.0, allow_headerless=True)
        self.ts.registerCallback(self.callback_no_mask)

        # Create a thread for displaying images
        self.display_thread = threading.Thread(target=self.display_images_thread)
        self.display_thread.daemon = True
        self.display_thread.start()

        # Create a thread for displaying masks
        self.mask_thread = threading.Thread(target=self.display_masks_thread)
        self.mask_thread.daemon = True
        self.mask_thread.start()

        rospy.loginfo("Image Display Node Started")

    def display_images_thread(self):
        # This thread will handle the regular image display
        rate = rospy.Rate(DISPLAY_RATE)
        while not rospy.is_shutdown():
            try:
                if DISPLAY_RGB and self.rgb_image is not None and self.rgb_image.size > 0:
                    cv2.imshow("RGB Image", self.rgb_image)
                if DISPLAY_DEPTH and self.depth_image is not None and self.depth_image.size > 0:
                    cv2.imshow("Depth Image", self.depth_image)
                if DISPLAY_REG and self.reg_image is not None and self.reg_image.size > 0:
                    cv2.imshow("Registered Image", self.reg_image)
                cv2.waitKey(1)
            except Exception as e:
                rospy.logerr(f"Error displaying images: {e}")
            rate.sleep()

    def display_masks_thread(self):
        # This thread will handle the mask display
        while not rospy.is_shutdown() and DISPLAY_MASKS:
            try:
                overlay = self.mask_image.copy() if self.mask_image is not None else None
                alpha = 0.5  # Transparency factor

                for mask, centroid, phrase, depth in zip(self.masks, self.centroids, self.phrases, self.depth_vals):
                    if overlay is None:
                        break

                    mask_rgb = np.zeros_like(self.mask_image)
                    mask_rgb[mask] = [0, 255, 0]  # Green color for mask

                    # Overlay mask on the image
                    cv2.addWeighted(mask_rgb, alpha, overlay, 1 - alpha, 0, overlay)

                    # Draw centroid
                    cv2.circle(overlay, (centroid[0], centroid[1]), 5, (0, 0, 255), -1)  # Red color for centroid

                    # Put text for phrase and depth value
                    cv2.putText(overlay, f"{phrase}, {depth:.2f}m", (centroid[0] + 10, centroid[1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

                if overlay is not None:
                    cv2.imshow("Mask Overlay Image", overlay)
                    cv2.waitKey(1)

            except Exception as e:
                rospy.logerr(f"Error displaying masks: {e}")
            rospy.sleep(DISPLAY_INTERVAL)

    def process_mask_data(self, mask_data):
        try:
            # This will display the mask data anytime it is called (independent from other image displays)
            if not DISPLAY_MASKS:  # don't process and display if display_masks is off
                return
            
            rospy.loginfo("Processing mask data")
            # Extract masks

            # Empty arrays
            self.phrases = []
            self.centroids = []
            self.depth_vals = []
            self.logits = []
            self.masks = []  # boolean array
            self.mask_image = None  # RGB image accompanying mask data

            # Convert data back to accessible data types
            for temp_mask in mask_data.mask_data:
                temp_phrase = temp_mask.phrase
                centroid_loc = (int(temp_mask.centroid.x), int(temp_mask.centroid.y))  # Convert to integer
                depth_val = temp_mask.centroid.z
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
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
        except Exception as e:
            rospy.logerr(f"Error processing mask data: {e}")

    def callback_no_mask(self, rgb_msg, depth_msg, reg_msg):
        try:
            # Convert the rgb and depth images to OpenCV format
            self.rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            self.depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="32FC1")
            self.reg_image = self.bridge.imgmsg_to_cv2(reg_msg, desired_encoding="bgr8")

        except CvBridgeError as e:
            rospy.logerr(f"Failed to convert images: {e}")
        except Exception as e:
            rospy.logerr(f"Error in callback_no_mask: {e}")
    
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