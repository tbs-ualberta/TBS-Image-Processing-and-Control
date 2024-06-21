# Authored by: Andor Siegers

# using transformers-4.27.4 and huggingface-hub-0.16.4
# DO NOT INSTALL IN CONDA ENVIRONMENT: this caused many incompatibility issues when tested

import numpy as np
# -------------------------------------------- User defined constants ------------------------------------------------

# Processing frequency: this is the max frequency. It should also be noted that the image recognition is not
# included in this time, so each cycle will be around 0.5s minimum, regardless of the set rate
PROCESSING_RATE = 1 # in Hz

# The minimum confidence the model must have to use the object as a target
TARGET_CONFIDENCE_THRESHOLD = 0.7

# Select whether the output should be printed to the terminal or not
PRINT_OUTPUT = False

# --------------------------------------------------------------------------------------------------------------------

# -------------------------------------------- Calculated constants --------------------------------------------------

# Calculate the process interval in seconds
PROCESSING_INTERVAL = 1.0 / PROCESSING_RATE

# --------------------------------------------------------------------------------------------------------------------

import rospy
from sensor_msgs.msg import Image as MsgImg
from geometry_msgs.msg import Point
from image_processing.msg import MaskArray
import message_filters
from cv_bridge import CvBridge, CvBridgeError
import os
import sys
import time
import requests
from PIL import Image as PilImg
from lang_sam import LangSAM
sys.path.append(os.path.join(os.path.dirname(__file__)))
from process_helper import map_rgb_to_depth, calculate_centroids, find_target, convert_to_MaskArray
import warnings
from transformers import logging
from kinect_pub.srv import GetCameraInfo

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

        # Get camera instrinsic data from service
        get_camera_info = rospy.ServiceProxy('/rgbd_out/get_camera_info', GetCameraInfo)
        camera_info = get_camera_info()

        # Initialize intrinsic matrices
        self.rgb_intrinsics = np.zeros((3,3))
        self.depth_intrinsics = np.zeros((3,3))
        self.depth_dist_coeffs = np.zeros(5)
        self.extrinsic_matrix = np.zeros((3,3))
        
        if camera_info:
            # Construct the intrinsic matrices
            self.rgb_intrinsics = np.array([[camera_info.rgb_fx, 0, camera_info.rgb_cx],
                                    [0, camera_info.rgb_fy, camera_info.rgb_cy],
                                    [0, 0, 1]])
            
            self.depth_intrinsics = np.array([[camera_info.ir_fx, 0, camera_info.ir_cx],
                                        [0, camera_info.ir_fy, camera_info.ir_cy],
                                        [0, 0, 1]])
            self.depth_dist_coeffs = np.array([0.0893804, -0.272566, 0, 0, 0.0958438])

            # Construct extrinsic matrix
            R = np.eye(3)  # Rotation matrix (3x3)
            T = np.array([0, 0, 0])  # Translation vector (3x1)
            self.extrinsic_matrix = np.eye(4)
            self.extrinsic_matrix[:3, :3] = R
            self.extrinsic_matrix[:3, 3] = T

        # Define publisher for mask data
        self.mask_pub = rospy.Publisher("process/mask_data", MaskArray, queue_size=10)

        # Define publisher for target position data
        self.target_pub = rospy.Publisher("process/target", Point, queue_size=10) # in px

        # Define subscribers for depth and rgb topics
        self.rgb_sub = message_filters.Subscriber('/rgbd_out/rgb', MsgImg)
        self.depth_sub = message_filters.Subscriber('/rgbd_out/depth_raw', MsgImg)

        # Synchronize rgb and depth topics
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

    def process_images(self, event):
        if self.rgb_image is not None and self.depth_image is not None:
            # rospy.loginfo("Processing images")
            try:
                # -----------------------------Process Images----------------------------------

                mask_array = MaskArray() # for publishing mask and centroid data

                # Convert image to PIL format
                image_pil = PilImg.fromarray(self.rgb_image)
                # Save image being processed to save alongside masks
                prediction_image_rgb = self.rgb_image
                prediction_image_depth = self.depth_image

                # Resize PIL image
                w, h = image_pil.size
                new_w, new_h = w // 1, h // 1
                image_pil = image_pil.resize((new_w, new_h), PilImg.ANTIALIAS)

                # Save start time (to evaluate process time)
                time_start = time.time()

                # Calculate masks and bounding boxes for images
                masks, boxes, phrases, logits = self.model.predict(image_pil, self.prompt)

                # Initialize target values
                target_pos = Point(-1,-1,-1)

                # Convert masks to numpy arrays
                masks_np = [mask.squeeze().cpu().numpy() for mask in masks]
                    
                # Calculate the centroids of each mask
                centroids_as_pixels = []
                centroids_as_pixels = calculate_centroids(masks_np)

                # Calculate corresponding depth values from RGB pixel coordinates
                depth_vals = []
                depth_vals = [map_rgb_to_depth(x, y, self.depth_image, self.rgb_intrinsics, self.depth_intrinsics, 
                                                self.depth_dist_coeffs, self.extrinsic_matrix) for x, y in centroids_as_pixels]

                # Find target
                target_pos = find_target(self.target, TARGET_CONFIDENCE_THRESHOLD, centroids_as_pixels, phrases, depth_vals, logits)

                # Convert to correct message type for publishing
                mask_array = convert_to_MaskArray(
                    centroids_as_pixels, depth_vals, phrases, logits, masks_np, prediction_image_rgb, prediction_image_depth)
                
                # Publish target values
                self.target_pub.publish(target_pos)

                # Publish mask data
                self.mask_pub.publish(mask_array)

                if PRINT_OUTPUT:
                    # Clear the console for new output
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
                        for i, (cent, depth, phr, log) in enumerate(zip(centroids_as_pixels, depth_vals, phrases, logits)):
                            # Print info
                            rospy.loginfo(f"Mask {i+1}: {phr} (confidence of {log}")
                            rospy.loginfo(f"Centroid at: {cent}, Depth value: {depth} mm")
                            print("-------------------------------------------------------------------------------------------------")

                    rospy.loginfo(f"Total processing time: {time.time() - time_start}")
                    print("-------------------------------------------------------------------------------------------------")

            except (requests.exceptions.RequestException, IOError) as e:
                rospy.logerr(f"Error: {e}")
            
            # -----------------------------------------------------------------------------

            # Reset images after processing
            self.rgb_image = None
            self.depth_image = None

    def callback(self, rgb_msg, depth_msg):
        try:
            # Convert the rgb and depth images to OpenCV format
            self.rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            self.depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="32FC1")

        except CvBridgeError as e:
            rospy.logerr(f"Failed to convert images: {e}")

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