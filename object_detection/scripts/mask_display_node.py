#!/usr/bin/env python3

import rospy
from object_detection.msg import MaskArray, MaskData
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class ImageSaverProcessor:
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
        self.mask_image = None # RGB image accompanying mask data

        # Define subscriber for masks
        self.mask_sub = rospy.Subscriber('/process/mask_data', MaskArray, self.convert_mask_data)

        # Define publisher to publish mask image
        self.mask_img_pub = rospy.Publisher("process/mask_img", Image, queue_size=10)

        rospy.loginfo("Mask Display Node Started")

    def convert_mask_data(self, mask_data):
        try:
            # This will display the mask data anytime it is called

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

            self.display_masks()

        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
        except Exception as e:
            rospy.logerr(f"Error processing mask data: {e}")
    
    def display_masks(self):
        try:
            if self.mask_image is None:
                rospy.logwarn("mask_image is None, skipping display")
                return
            
            overlay = self.mask_image.copy()
            alpha = 0.5  # Transparency factor
            
            mask_sum = np.zeros_like(self.mask_image)

            # Convert mask to single array. This allows for even application of colour and the image is not too darkened by multiple masks being overlaid.
            for mask, logit in zip(self.masks, self.logits):
                mask_scaled = np.zeros_like(self.mask_image)
                # Scale proportionally to confidence value
                scaled_val = logit * 225
                mask_scaled[mask] = [0, scaled_val, 0] # change these values to change colour
                # Combine arrays
                mask_sum = np.maximum(mask_sum, mask_scaled)

            # Overlay mask on the image
            cv2.addWeighted(mask_sum, alpha, overlay, 1 - alpha, 0, overlay)

            for centroid, phrase, depth, logit in zip(self.centroids, self.phrases, self.depth_vals, self.logits):
                # Draw centroid
                cv2.circle(overlay, (centroid[0], centroid[1]), 5, (0, 0, 255), -1)  # Red color for centroid

                # Put text for phrase and depth value
                cv2.putText(overlay, f"{phrase}, {depth:.2f}m", (centroid[0] + 10, centroid[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

            # Publish image to ROS topic for image saving
            self.mask_img_pub.publish(self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8"))

            # Display image in window
            cv2.imshow("Mask Overlay Image", overlay)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f"Error displaying masks: {e}")
    
    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        image_saver_processor = ImageSaverProcessor()
        image_saver_processor.spin()
    except rospy.ROSInterruptException:
        pass