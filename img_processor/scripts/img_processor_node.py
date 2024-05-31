# Authored by: Andor Siegers

# using transformers-4.27.4 and huggingface-hub-0.16.4
# DO NOT INSTALL IN CONDA ENVIRONMENT: this caused many incompatibility issues

# -------------------------------------------- User defined constants ------------------------------------------------

# This specifies the prompt which the model masks
PROMPT = "person" # multiple objects can be detected using a '.' as a separator
# PROMPT = "cat"

# Specifies target. This allows for detection of multiple objects but only targetting of one
TARGET = "person" # must be contained in the prompt

# Processing frequency: this is the max frequency. It should also be noted that the image recognition is not
# included in this time, so it will add around 0.5s minimum to each cycle, regardless of the set rate
PROCESSING_RATE = 2 # in Hz

# --------------------------------------------------------------------------------------------------------------------

# -------------------------------------------- Calculated constants --------------------------------------------------

# Calculate the process interval in seconds
PROCESSING_INTERVAL = 1.0 / PROCESSING_RATE

# --------------------------------------------------------------------------------------------------------------------

import rospy
from std_msgs.msg import Int16, Float32
from sensor_msgs.msg import Image as MsgImg
from geometry_msgs.msg import Point
from img_processor.msg import MaskArray, MaskData
import message_filters
from cv_bridge import CvBridge, CvBridgeError
import os
import time
import numpy as np
import requests
from PIL import Image as PilImg
from lang_sam import LangSAM
from process_helper import ProcessingResults
from process_helper import map_rgb_to_depth
import warnings
from transformers import logging

# Suppress unimportant messages printing to the console
warnings.filterwarnings("ignore", category=UserWarning)
warnings.filterwarnings("ignore", message="torch.meshgrid: in an upcoming release")
logging.set_verbosity_error()
class ImageProcessor:
    def __init__(self):
        self.bridge = CvBridge()

        # Initialize node
        rospy.init_node('image_processor', anonymous=True)
        rospy.on_shutdown(self.cleanup)

        # Define publisher for mask data
        self.mask_pub = rospy.Publisher("process/mask_data", MaskArray, queue_size=10)

        # Publish position data to ROS topics for use with control algorithm
        self.target_pub_x = rospy.Publisher("process/target/x", Int16, queue_size=10) # in px
        self.target_pub_y = rospy.Publisher("process/target/y", Int16, queue_size=10) # in px
        self.target_pub_depth = rospy.Publisher("process/target/depth", Float32, queue_size=10) # in mm

        # Define subscribers for depth and rgb topics
        self.rgb_sub = message_filters.Subscriber('/kinect2/rgb', MsgImg)
        self.depth_sub = message_filters.Subscriber('/kinect2/depth_raw', MsgImg)

        # Synchronize the topics
        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], 10, 1.0)
        self.ts.registerCallback(self.callback)

        # Initialize images
        self.rgb_image = None
        self.depth_image = None

        # Initialize model
        self.model = LangSAM(sam_type = "vit_b")

        # Set up timers to call process function
        self.processing_timer = rospy.Timer(rospy.Duration(PROCESSING_INTERVAL), self.process_images)
        
        rospy.loginfo("Image Processor Node Started")

    def callback(self, rgb_msg, depth_msg):
        try:
            # Convert the rgb and depth images to OpenCV format
            self.rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            self.depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="32FC1")
            
        except CvBridgeError as e:
            rospy.logerr(f"Failed to convert images: {e}")

    def process_images(self, event):
        if self.rgb_image is not None and self.depth_image is not None:
            # rospy.loginfo("Processing images")
            try:
                # -----------------------------Process Images----------------------------------

                mask_array = MaskArray() # for publishing mask and centroid data

                # convert image to PIL format and resize
                image_pil = PilImg.fromarray(self.rgb_image)
                w, h = image_pil.size
                new_w, new_h = w // 1, h // 1
                image_pil = image_pil.resize((new_w, new_h), PilImg.ANTIALIAS)

                # Save start time (to evaluate process time)
                time_start = time.time()

                # Calculate masks and bounding boxes for images
                masks, boxes, phrases, logits = self.model.predict(image_pil, PROMPT)

                # Clear the console for new output
                os.system('clear')
                rospy.logdebug(f"Inference time: {time.time() - time_start}")

                if len(masks) == 0:
                    # If no objects detected
                    rospy.loginfo(f"No objects of the '{PROMPT}' prompt detected in image.")

                    # Add rgb image to array data
                    mask_array.rgb_img = self.bridge.cv2_to_imgmsg(self.rgb_image, encoding="bgr8")

                    # Publish centroid array
                    self.mask_pub.publish(mask_array)

                    # if object is not detected in frame, all values will be -1
                    self.target_pub_x.publish(-1)
                    self.target_pub_y.publish(-1)
                    self.target_pub_depth.publish(-1)

                else:
                    # If at least one object detected

                    # Create results object
                    results = ProcessingResults(self.rgb_image, masks, boxes, phrases, logits)
                    
                    # Calculate the centroids of each map
                    depth_vals = []
                    centroids = results.find_object_centroids()
                    centroids_as_pixels = [(int(x), int(y)) for y, x in centroids]

                    # Calculate corresponding depth values from RGB pixel coordinates
                    depth_vals = [map_rgb_to_depth(x, y, self.depth_image) for x, y in centroids_as_pixels]

                    # Print calculated data to console
                    print("-------------------------------------------------------------------------------------------------")
                    for i, (cent, depth, phr, log) in enumerate(zip(centroids_as_pixels, depth_vals, phrases, logits)):
                        # Print info
                        rospy.loginfo(f"Mask {i+1}: {phr} (confidence of {log}")
                        rospy.loginfo(f"Centroid at: {cent}, Depth value: {depth} mm")
                        print("-------------------------------------------------------------------------------------------------")

                    # Add mask data to array for publishing
                    for centroid, depth_val, phrase, logit, mask in zip(centroids_as_pixels, depth_vals, phrases, logits, results.masks_np):
                        temp_mask = MaskData()
                        temp_mask.phrase = phrase
                        temp_mask.centroid = Point(centroid[0], centroid[1], depth_val) # the z part of the point is depth (in mm)
                                                                                        # this should not be confused for a point in
                                                                                        # 3d cartesian space, as x and y are in px
                        temp_mask.logit = logit
                        mask_image = (mask * 255).astype(np.uint8)
                        temp_mask.mask = self.bridge.cv2_to_imgmsg(mask_image, encoding="mono8") 

                        mask_array.mask_data.append(temp_mask)

                    # Add rgb image to array data
                    mask_array.rgb_img = self.bridge.cv2_to_imgmsg(self.rgb_image, encoding="bgr8")

                    # Selects which object to target. This can be implemented in different ways,
                    # but for now, it will just select the closest object.
                    target_x = -1
                    target_y = -1
                    target_depth = -1
                    min_depth = 100000
                    
                    # Find minimum depth of nearest target
                    for temp_centroid, temp_phrase, temp_depth in zip(centroids_as_pixels, phrases, depth_vals):
                        if TARGET in temp_phrase and temp_depth < min_depth:
                            target_x = temp_centroid[0]
                            target_y = temp_centroid[1]
                            target_depth = temp_depth
                            min_depth = temp_depth

                    # Publish mask data
                    self.mask_pub.publish(mask_array)
                    
                    # Publish target values
                    self.target_pub_x.publish(target_x)
                    self.target_pub_y.publish(target_y)
                    self.target_pub_depth.publish(target_depth)

                rospy.loginfo(f"Total processing time: {time.time() - time_start}")
                print("-------------------------------------------------------------------------------------------------")

            except (requests.exceptions.RequestException, IOError) as e:
                rospy.logerr(f"Error: {e}")
            
            # -----------------------------------------------------------------------------

            # Reset images after processing
            self.rgb_image = None
            self.depth_image = None

    def cleanup(self):
        rospy.loginfo("Shutting down Image Processor Node")

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        image_processor = ImageProcessor()
        image_processor.spin()
    except rospy.ROSInterruptException:
        pass

# TODO sometimes centroid point does not have depth value.
# Maybe add algorithm to find closest point with depth info and use that (if no depth at exact pixel)?