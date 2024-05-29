# using transformers-4.27.4 and huggingface-hub-0.16.4
# DO NOT INSTALL IN CONDA ENVIRONMENT: this caused many incompatibility issues

import rospy
from sensor_msgs.msg import Image as MsgImg
import message_filters
import cv2
from cv_bridge import CvBridge, CvBridgeError
import os
import time
import shutil
import numpy as np
import imageio
import requests
from PIL import Image as PilImg
from lang_sam import LangSAM
from process_helper import ProcessingResults
from process_helper import map_rgb_to_depth

# Suppress unimportant messages printing to the console
import warnings
from transformers import logging

warnings.filterwarnings("ignore", category=UserWarning)
warnings.filterwarnings("ignore", message="torch.meshgrid: in an upcoming release")
logging.set_verbosity_error()

# -------------------------------------------- User defined constants ------------------------------------------------

# Directory to save images
SAVE_PATH = r'/home/wmm'

# Save folder name
SAVE_FOLDER_NAME = 'kinect_images'

# Selects whether files should process or not
PROCESS_IMAGES = True

# Processing frequency
PROCESSING_RATE = 0.2 # in Hz

# Select whether images should save or not
SAVE_IMAGES = False

# Frequency of images being saved
SAVE_RATE = 1 # in Hz

# This specifies the prompt which the model masks
PROMPT = "monitor"

# --------------------------------------------------------------------------------------------------------------------

# -------------------------------------------- Calculated constants --------------------------------------------------

# Calculate the process interval in seconds
PROCESSING_INTERVAL = 1.0 / PROCESSING_RATE

# Calculate the save interval in seconds
SAVE_INTERVAL = 1.0 / SAVE_RATE

# --------------------------------------------------------------------------------------------------------------------

class ImageSaverProcessor:
    def __init__(self):
        self.bridge = CvBridge()

        # Clear the save directory
        if SAVE_IMAGES:
            self.clear_directory(SAVE_PATH)

        # Initialize node
        rospy.init_node('image_saver_processor', anonymous=True)
        rospy.on_shutdown(self.cleanup)

        # for file saving
        self.last_save_time = rospy.Time.now()
        self.save_count = 1

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

        # Set up a timers to call save and process functions
        if PROCESS_IMAGES:
            self.processing_timer = rospy.Timer(rospy.Duration(PROCESSING_INTERVAL), self.process_images)
        if SAVE_IMAGES:
            self.save_timer = rospy.Timer(rospy.Duration(SAVE_INTERVAL), self.save_images)

        rospy.loginfo("Image Processor Node Started")

    def callback(self, rgb_msg, depth_msg):
        try:
            # Convert the rgb and depth images to OpenCV format
            self.rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            self.depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="32FC1")
        except CvBridgeError as e:
            rospy.logerr(f"Failed to convert images: {e}")

    def clear_directory(self, path):
        # Remove all files in the save_folder_name directory (if it exists)
        if os.path.isdir(os.path.join(path, SAVE_FOLDER_NAME)):
            shutil.rmtree(os.path.join(path, SAVE_FOLDER_NAME))
            rospy.loginfo("Image Directory Cleared")

        # create new folders to save to
        os.chdir(path)
        os.mkdir(SAVE_FOLDER_NAME)
        os.chdir(os.path.join(SAVE_PATH, SAVE_FOLDER_NAME))
        # only makes folder if saving for corresponding image type is ON
        os.mkdir('rgb')
        os.mkdir('depth')
    
    def save_images(self, event):
        # Saves images to computer
        # TODO this skips images a lot, so it should be changed, but it works for now
        if self.rgb_image is not None and self.depth_image is not None:
            # rgb
            os.chdir(os.path.join(SAVE_PATH, SAVE_FOLDER_NAME, "rgb"))
            path_str_rgb = "rgb_" + str(self.save_count).zfill(6) + ".png"
            cv2.imwrite(str(path_str_rgb), self.rgb_image)

            # depth
            os.chdir(os.path.join(SAVE_PATH, SAVE_FOLDER_NAME, "depth"))
            path_str_depth = "depth_" + str(self.save_count).zfill(6) + ".tiff"
            imageio.imwrite(str(path_str_depth), self.depth_image)

            self.save_count += 1

            rospy.loginfo(f"Saved images at index" + str(self.save_count).zfill(6))


    def process_images(self, event):
        if self.rgb_image is not None and self.depth_image is not None:
            rospy.loginfo("Processing images")
            try:
                # -----------------------------Process Images----------------------------------
                
                # convert image to PIL format and resize
                image_pil = PilImg.fromarray(self.rgb_image)
                w, h = image_pil.size
                new_w, new_h = w // 1, h // 1
                image_pil = image_pil.resize((new_w, new_h), PilImg.ANTIALIAS)

                # Save start time (to evaluate process time)
                time_start = time.time()
                masks, boxes, phrases, logits = self.model.predict(image_pil, PROMPT)
                rospy.loginfo(f"inference time: {time.time() - time_start}")

                if len(masks) == 0:
                    # If no objects detected
                    rospy.loginfo(f"No objects of the '{PROMPT}' prompt detected in image.")

                else:
                    # If at least one object detected

                    # Create results object
                    results = ProcessingResults(self.rgb_image, masks, boxes, phrases, logits)

                    # Print the bounding boxes, phrases, and logits
                    results.print_bounding_boxes()
                    results.print_detected_phrases()
                    results.print_logits()

                    # Display the original image and masks side by side
                    results.display_image_with_masks()

                    # Display the image with bounding boxes and confidence scores
                    results.display_image_with_boxes()


                    rgb_x = 960
                    rgb_y = 540
                    
                    # Calculate corresponding depth value of from RGB pixel coordinates
                    depth_val = map_rgb_to_depth(rgb_x, rgb_y, self.depth_image)
                    
                    rospy.loginfo("The depth value for the point (" + str(rgb_x) + ", " + str(rgb_y) + ") is " + str(depth_val) + "mm.")

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
        image_saver_processor = ImageSaverProcessor()
        image_saver_processor.spin()
    except rospy.ROSInterruptException:
        pass

# TODO add image processing to find objects in frame
# TODO process the distance to the center of each object