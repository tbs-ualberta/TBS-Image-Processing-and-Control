#!/usr/bin/env python3
# Authored by: Andor Siegers

import rclpy
from rclpy.node import Node
from image_processing.msg import MaskArray
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from utils import unpack_MaskArray
import traceback

DISPLAY_PHRASE = True       # Selects whether the phrase should be displayed along with the masked image
DISPLAY_CENT_DEPTH = True   # Selects whether the depth of the centroid should be displayed
DISPLAY_AVG_DEPTH = True    # Selects whether the average depth of the mask should be displayed
SCALE_FACTOR = 0.85         # Determines how much the image should be scaled, as it doesn't fit in a 1080p window natively

class MaskDisplay(Node):
    def __init__(self):
        super().__init__('mask_display')
        self.bridge = CvBridge()

        # Initialize mask data arrays
        self.phrases = []
        self.centroids = []
        self.centroid_depths = []
        self.logits = []
        self.avg_depths = []
        self.masks = []  # boolean array
        self.rgb_image = None  # RGB image accompanying mask data
        self.depth_image = None  # Depth image accompanying mask data

        # Define subscriber for masks
        self.mask_sub = self.create_subscription(MaskArray, '/process/mask_data', self.mask_callback, 10)

        # Define publisher to publish mask image
        self.mask_img_pub = self.create_publisher(Image, '/process/mask_img', 10)

        self.get_logger().info("Mask Display Node Started")

    def mask_callback(self, mask_data):
        # Runs whenever new mask data is received
        # Convert raw mask data from message to usable datatypes
        self.phrases, self.centroids, self.centroid_depths, self.logits, self.avg_depths, self.masks, self.rgb_image, self.depth_image = unpack_MaskArray(mask_data)

        # Display the mask data overlayed over rgb image
        self.display_masks()

    def display_masks(self):
        try:
            if self.rgb_image is None:
                self.get_logger().warn("rgb_image is None, skipping display")
                return

            overlay = self.rgb_image.copy()
            alpha = 0.5  # Transparency factor

            mask_sum = np.zeros_like(self.rgb_image)

            # Convert mask to single array. This allows for even application of colour and the image is not too darkened by multiple masks being overlaid.
            for mask, logit in zip(self.masks, self.logits):
                mask_scaled = np.zeros_like(self.rgb_image)
                # Scale proportionally to confidence value
                scaled_val = [0, 255 * logit, 0]  # RGB values that determine mask colour
                mask_scaled[mask] = scaled_val
                # Combine arrays
                mask_sum = np.maximum(mask_sum, mask_scaled)

            # Overlay mask on the image
            cv2.addWeighted(mask_sum, alpha, overlay, 1 - alpha, 0, overlay)

            for centroid, phrase, cent_depth, logit, avg_depth in zip(self.centroids, self.phrases, self.centroid_depths, self.logits, self.avg_depths):
                # Draw centroid
                cv2.circle(overlay, (centroid[0], centroid[1]), 5, (0, 0, 255), -1)  # Red colour for centroid

                if DISPLAY_PHRASE:
                    text_to_show = phrase
                    if DISPLAY_CENT_DEPTH or DISPLAY_AVG_DEPTH:
                        text_to_show += ", "

                if DISPLAY_AVG_DEPTH:
                    text_to_show += f"avg: {avg_depth:.2f}m"

                    if DISPLAY_CENT_DEPTH:
                        text_to_show += ", "

                if DISPLAY_CENT_DEPTH:
                    text_to_show += f"cent: {cent_depth:.2f}m"

                # Display text beside centroid
                if DISPLAY_PHRASE or DISPLAY_CENT_DEPTH or DISPLAY_AVG_DEPTH:
                    cv2.putText(overlay, text_to_show, (centroid[0] + 10, centroid[1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

            # Publish image to ROS topic for image saving
            self.mask_img_pub.publish(self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8"))

            overlay_small = cv2.resize(overlay, (0, 0), fx=SCALE_FACTOR, fy=SCALE_FACTOR)

            # Display image in window
            cv2.imshow("Mask Overlay Image", overlay_small)

            # Uncomment to check if mask is displaying properly - for testing
            # cv2.imshow("Depth Image", self.depth_image)

            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error displaying masks: {e}")
            self.get_logger().error(traceback.format_exc())

    def spin(self):
        rclpy.spin(self)


def main(args=None):
    rclpy.init(args=args)
    try:
        image_saver_processor = MaskDisplay()
        image_saver_processor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
