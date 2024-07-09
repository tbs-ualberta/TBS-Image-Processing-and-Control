# Authored by: Andor Siegers

# using transformers-4.27.4 and huggingface-hub-0.16.4
# DO NOT INSTALL IN CONDA ENVIRONMENT: this caused many incompatibility issues when tested

# -------------------------------------------- User defined constants ------------------------------------------------

# Processing frequency: this is the max frequency. It should also be noted that the image recognition is not
# included in this time, so each cycle will be around 0.5s minimum, regardless of the set rate
PROCESSING_RATE = 1 # in Hz

# The minimum confidence the model must have to use the object as a target
TARGET_CONFIDENCE_THRESHOLD = 0.1

# Select whether the output should be printed to the terminal or not
PRINT_OUTPUT = False

# Select whether the output should clear the console before outputting each cycle data
CLEAR_OUTPUT = True

# --------------------------------------------------------------------------------------------------------------------

# -------------------------------------------- Calculated constants --------------------------------------------------

# Calculate the process interval in seconds
PROCESSING_INTERVAL = 1.0 / PROCESSING_RATE

# --------------------------------------------------------------------------------------------------------------------

import rospy
from geometry_msgs.msg import Point
from image_processing.msg import MaskArray
from cv_bridge import CvBridge
import os
import sys
import requests
from PIL import Image as PilImg
from lang_sam import LangSAM
sys.path.append(os.path.join(os.path.dirname(__file__)))
from utils import calculate_centroids, get_avg_depth, find_target, convert_to_MaskArray, unpack_RegistrationData
import warnings
from transformers import logging
from kinect_pub.msg import RegistrationData

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

        # This specifies the prompt which the model masks
        self.prompt = rospy.get_param('prompt', 'person.chair')# multiple objects can be detected using a '.' as a separator

        # Specifies target. This allows for detection of multiple objects but only targetting of one
        self.target = rospy.get_param('target', 'person') # must be contained in the prompt
        
        # Define publisher for mask data
        self.mask_pub = rospy.Publisher("process/mask_data", MaskArray, queue_size=10)

        # Define publisher for target position data
        self.target_pub = rospy.Publisher("process/target", Point, queue_size=10) # in px

        # Define subscribers for registration data (which includes rgb and depth images)
        self.reg_sub = rospy.Subscriber('rgbd_out/reg_data', RegistrationData, self.callback)

        # Initialize registration data message
        self.reg_data = None

        # Initialize model
        self.model = LangSAM(sam_type = "vit_b")

        # Set up timers to call process function
        self.processing_timer = rospy.Timer(rospy.Duration(PROCESSING_INTERVAL), self.process_images)
        
        rospy.loginfo("Image Processor Node Started")

    def process_images(self, event):
        if self.reg_data is None:
            return
        
        # rospy.loginfo("Processing images")
        try:
            # Save start time (to evaluate process time)
            process_time_start = rospy.Time.now()

            prediction_reg_data = self.reg_data
            # Unpack data from message data type
            rgb_image, depth_image, undistorted_image, __, prediction_bigdepth_image, __, image_time = unpack_RegistrationData(prediction_reg_data)

            # For publishing mask and centroid data
            mask_array = MaskArray() 

            # Convert image to PIL format
            image_pil = PilImg.fromarray(rgb_image)
            
            # Resize PIL image
            w, h = image_pil.size
            new_w, new_h = w // 1, h // 1
            image_pil = image_pil.resize((new_w, new_h), PilImg.ANTIALIAS)

            # Calculate masks and bounding boxes for images
            masks, boxes, phrases, logits = self.model.predict(image_pil, self.prompt)

            # Initialize target values
            target_pos = Point(-1,-1,-1)

            # Convert masks to numpy arrays
            masks_np = [mask.squeeze().cpu().numpy() for mask in masks]
                
            # Calculate the centroids of each mask
            centroids_as_pixels = []
            centroids_as_pixels = calculate_centroids(masks_np)

            # Find corresponding RGB values from pixel coordinates
            centroid_depths = []
            centroid_depths = [prediction_bigdepth_image[y+1,x] for x, y in centroids_as_pixels]

            # Find average depth value of each mask
            avg_depths = []
            avg_depths = [get_avg_depth(mask, prediction_bigdepth_image) for mask in masks_np]

            # Find target
            target_pos = find_target(self.target, TARGET_CONFIDENCE_THRESHOLD, centroids_as_pixels, phrases, avg_depths, logits)

            # Convert to correct message type for publishing
            mask_array = convert_to_MaskArray(centroids_as_pixels, centroid_depths, phrases, logits, avg_depths, masks_np, prediction_reg_data)
            
            # Publish target values
            self.target_pub.publish(target_pos)

            # Publish mask data
            self.mask_pub.publish(mask_array)

            if PRINT_OUTPUT:
                # Clear the console for new output
                if CLEAR_OUTPUT:
                    os.system('clear')

                # Print data to console
                if len(masks) == 0:
                    # If no objects detected
                    rospy.loginfo(f"No objects of the '{self.prompt}' prompt detected in image.")
                    # if object is not detected in frame, all target values will be -1 (see above)

                else:
                    # If at least one object detected

                    # Print calculated data to console
                    print("-------------------------------------------------------------------------------------------------")
                    for i, (cent_coord, avg_depth, phr, log) in enumerate(zip(centroids_as_pixels, avg_depths, phrases, logits)):
                        # Print info
                        rospy.loginfo(f"Mask {i+1}: {phr} (confidence of {log}")
                        rospy.loginfo(f"Centroid at: {cent_coord}, Average depth value: {avg_depth} mm")
                        print("-------------------------------------------------------------------------------------------------")

                process_dif = rospy.Time.now() - process_time_start
                image_dif = rospy.Time.now() - image_time
                rospy.loginfo("Total processing time: %d.%09ds", process_dif.secs, process_dif.nsecs)
                rospy.loginfo("Total time since image taken: %d.%09ds", image_dif.secs, image_dif.nsecs)
                print("-------------------------------------------------------------------------------------------------")

        except (requests.exceptions.RequestException, IOError) as e:
            rospy.logerr(f"Error: {e}")

    def callback(self, reg_data):
        # Unpack registration data
        self.reg_data = reg_data

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