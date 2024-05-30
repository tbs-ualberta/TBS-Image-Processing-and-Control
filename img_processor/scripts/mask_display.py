#!/usr/bin/env python3

import rospy
from img_processor.msg import MaskArray, MaskData
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
        self.mask_sub = rospy.Subscriber('/process/mask_data', MaskArray, self.process_mask_data)

        rospy.loginfo("Image Display Node Started")

    def process_mask_data(self, mask_data):
        try:
            # This will display the mask data anytime it is called
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

            print(type(self.mask_image))

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

            if overlay.size == 0:
                rospy.logwarn("Overlay image has zero size, skipping display")
                return

            for mask, centroid, phrase, depth in zip(self.masks, self.centroids, self.phrases, self.depth_vals):
                if overlay is None:
                    rospy.logwarn("Overlay is None, breaking loop")
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
                if overlay.size > 0:
                    rospy.loginfo(f"Displaying overlay with size: {overlay.shape}")
                    cv2.imshow("Mask Overlay Image", overlay)
                    cv2.waitKey(1)
                    pass
                else:
                    rospy.logwarn("Overlay image has zero size, skipping display")
            else:
                rospy.logwarn("Overlay image is None, skipping display")

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

# TODO figure out why the registered image is the same as the rgb image
# TODO add visualization for the masking algorithm, centroid locations, and depth values
