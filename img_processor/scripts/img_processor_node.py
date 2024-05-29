# using transformers-4.27.4 and huggingface-hub-0.16.4
# DO NOT INSTALL IN CONDA ENVIRONMENT: this caused many incompatibility issues

import rospy
from sensor_msgs.msg import Image as MsgImg
from geometry_msgs.msg import Point
from img_processor.msg import MaskArray, MaskData
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

# Processing frequency: this is the max frequency. It should also be noted that the image recognition is not
# included in this time, so it will add around 0.5s minimum to each cycle, regardless of the set rate
PROCESSING_RATE = 1 # in Hz

# Select whether images should save or not
SAVE_IMAGES = False

# Frequency of images being saved
SAVE_RATE = 1 # in Hz

# This specifies the prompt which the model masks
PROMPT = "chair . human"

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

        # Initialize node
        rospy.init_node('image_saver_processor', anonymous=True)
        rospy.on_shutdown(self.cleanup)

        # Define publishers for masks, centroid locations, and centroid depths
        self.mask_pub = rospy.Publisher("process/mask_data", MaskArray, queue_size=10)

        # Define subscribers for depth and rgb topics
        self.rgb_sub = message_filters.Subscriber('/kinect2/rgb', MsgImg)
        self.depth_sub = message_filters.Subscriber('/kinect2/depth_raw', MsgImg)

        # Synchronize the topics
        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], 10, 1.0)
        self.ts.registerCallback(self.callback)

        # Initialize images
        self.rgb_image = None
        self.depth_image = None

        # Clear the save directory
        if SAVE_IMAGES:
            self.clear_directory(SAVE_PATH)

        # For file saving
        self.last_save_time = rospy.Time.now()
        self.save_count = 1

        # Initialize model
        self.model = LangSAM(sam_type = "vit_b")

        # Set up timers to call save and process functions
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

                    # Publish calculated data to ROS topic
                    for centroid, depth_val, phrase, logit, mask in zip(centroids_as_pixels, depth_vals, phrases, logits, results.masks_np):
                        # add mask data to array for publishing
                        temp_mask = MaskData()
                        temp_mask.phrase = phrase
                        temp_mask.centroid = Point(centroid[0], centroid[1], depth_val) # the z part of the point is depth (in mm)
                                                                                        # this should not be confused for a point in
                                                                                        # 3d cartesian space, as x and y are in px
                        temp_mask.logit = logit
                        mask_image = (mask * 255).astype(np.uint8)
                        temp_mask.mask = self.bridge.cv2_to_imgmsg(mask_image, encoding="mono8")

                        mask_array.data.append(temp_mask)

                    # publish centroid array
                    self.mask_pub.publish(mask_array)

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
        image_saver_processor = ImageSaverProcessor()
        image_saver_processor.spin()
    except rospy.ROSInterruptException:
        pass

# TODO sometimes centroid point does not have depth value.
# Maybe add algorithm to find closest point with depth info and use that (if no depth at exact pixel)?

# TODO add registered image back in (depth mapped over rgb)