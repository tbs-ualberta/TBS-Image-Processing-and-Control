# Authored by: Andor Siegers

# DO NOT INSTALL IN CONDA ENVIRONMENT: this caused many incompatibility issues when tested

# -------------------------------------------- User defined constants ------------------------------------------------

# Processing frequency: this is the max frequency. It should also be noted that the image recognition is not
# included in this time, so each cycle will be around 0.5s minimum, regardless of the set rate
PROCESSING_RATE = 1 # in Hz

# The minimum confidence the model must have to use the object as a target
TARGET_CONFIDENCE_THRESHOLD = 0.1

# If the processing should follow the specified rate or just do it as fast as possible
REGULATE_PROCESS_RATE = False

# --------------------------------------------------------------------------------------------------------------------

# -------------------------------------------- Calculated constants --------------------------------------------------

# Calculate the process interval in seconds
PROCESSING_INTERVAL = 1.0 / PROCESSING_RATE

# --------------------------------------------------------------------------------------------------------------------

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, CameraInfo
from process_msgs.msg import MaskArray
from cv_bridge import CvBridge
import message_filters
import os
import sys
import requests
from PIL import Image as PilImg
from lang_sam import LangSAM
sys.path.append(os.path.join(os.path.dirname(__file__)))
from utils import CameraParams, calculate_centroids, get_avg_depth, find_target, convert_to_MaskArray, get_point_xyz
import warnings
from transformers import logging
from rclpy.duration import Duration
import numpy as np

# Suppress unimportant messages printing to the console
warnings.filterwarnings("ignore", category=UserWarning)
warnings.filterwarnings("ignore", message="torch.meshgrid: in an upcoming release")
logging.set_verbosity_error()

import torch
print(torch.__version__)
print(torch.cuda.is_available())

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.bridge = CvBridge()

        # Specifies the prompt which the model masks
        self.prompt = self.declare_parameter('prompt', 'person').get_parameter_value().string_value # multiple objects can be detected using a '.' as a separator

        # Specifies target. This allows for detection of multiple objects but only targetting of one
        self.target = self.declare_parameter('target', 'person').get_parameter_value().string_value # must be contained in the prompt
        
        # Specifies whether to print the output to the terminal or not
        self.PRINT_OUTPUT = self.declare_parameter('print_output', 'false').get_parameter_value().string_value
        self.PRINT_OUTPUT = (self.PRINT_OUTPUT.lower() == 'true')

        # Specifies whether the output should clear the console before outputting each cycle data
        self.CLEAR_OUTPUT = self.declare_parameter('clear_output', 'true').get_parameter_value().string_value
        self.CLEAR_OUTPUT = (self.CLEAR_OUTPUT.lower() == 'false')

        # Specifies RGB image topic name
        self.rgb_image_topic = self.declare_parameter('rgb_image_topic', 'zed/zed_node/rgb/image_rect_color').get_parameter_value().string_value # NOTE: this uses the rectified image, which may not be correct. It needs to be validated

        # Specifies Depth image topic name
        self.depth_img_topic = self.declare_parameter('depth_img_topic', 'zed/zed_node/depth/depth_registered').get_parameter_value().string_value

        # Debug: print the parameters to verify
        self.get_logger().info(f'prompt: {self.prompt}')
        self.get_logger().info(f'target: {self.target}')
        self.get_logger().info(f'print_output: {self.PRINT_OUTPUT}')
        self.get_logger().info(f'clear_output: {self.CLEAR_OUTPUT}')
        self.get_logger().info(f'rgb image topic: {self.rgb_image_topic}')
        self.get_logger().info(f'depth image topic: {self.depth_img_topic}')
        
        # Define publisher for mask data
        self.mask_pub = self.create_publisher(MaskArray, "process/mask_data", 10)

        # Define publisher for target position data
        self.target_pub = self.create_publisher(Point, "process/target", 10) # in px

        # Define subscribers for camera data (i.e. rgb and depth images and camera info)
        self.left_rgb_sub = message_filters.Subscriber(self, Image, self.rgb_image_topic)
        self.left_depth_reg_sub = message_filters.Subscriber(self, Image, self.depth_img_topic)
        self.cam_info_sub = self.create_subscription(CameraInfo,' /zed/zed_node/depth/camera_info', self.cam_info_callback)

        self.ts = message_filters.TimeSynchronizer([self.left_rgb_sub, self.left_depth_reg_sub], queue_size=10)
        self.ts.registerCallback(self.image_callback)

        self.rgb_image = None
        self.depth_image = None
        self.depth_array = None
        self.camera_params = CameraParams()

        # Initialize model
        self.model = LangSAM(sam_type = "vit_b")

        if REGULATE_PROCESS_RATE:
            # Set up timers to call process function
            self.processing_timer = self.create_timer(PROCESSING_INTERVAL, self.process_images)
        
        self.get_logger().info("Image Processor Node Started")

    def process_images(self):
        if self.rgb_image is None or self.depth_image is None:
            return
        
        try:
            # Save start time (to evaluate process time)
            process_time_start = self.get_clock().now()

            rgb_image = self.rgb_image
            depth_image = self.depth_image

            # For publishing mask and centroid data
            mask_array = MaskArray()

            # Convert depth image to depth array
            depth_array = np.array(depth_image, dtype=np.float32)

            # Convert image to PIL format
            image_pil = PilImg.fromarray(rgb_image)
            
            # Resize PIL image
            w, h = image_pil.size
            new_w, new_h = w // 1, h // 1
            image_pil = image_pil.resize((new_w, new_h), PilImg.ANTIALIAS)

            # Calculate masks and bounding boxes for images
            masks, boxes, phrases, logits = self.model.predict(image_pil, self.prompt)

            # Initialize target values
            target_pos = Point(x=-1.0,y=-1.0,z=-1.0)

            # Convert masks to numpy arrays
            masks_np = [mask.squeeze().cpu().numpy() for mask in masks]
                
            # Calculate the centroids of each mask
            centroids_as_pixels = []
            centroids_as_pixels = calculate_centroids(masks_np)

            # Find each centroid's point in 3D space TODO
            centroids_cartesian = []
            for centroid_as_px in centroids_as_pixels:
                centroids_cartesian = get_point_xyz(centroid_as_px, depth_array, self.camera_params)

            # Find average depth value of each mask
            avg_depths = []
            avg_depths = [get_avg_depth(mask, depth_array) for mask in masks_np]

            # Find target
            target_pos = find_target(self.target, TARGET_CONFIDENCE_THRESHOLD, centroids_as_pixels, phrases, avg_depths, logits)

            # Convert to correct message type for publishing
            mask_array = convert_to_MaskArray(centroids_as_pixels, centroids, phrases, logits, avg_depths, masks_np, rgb_image, depth_image)
            
            # Publish target values
            self.target_pub.publish(target_pos)

            # Publish mask data
            self.mask_pub.publish(mask_array)

            if self.PRINT_OUTPUT:
                # Clear the console for new output
                if self.CLEAR_OUTPUT:
                    os.system('clear')

                # Print data to console
                if len(masks) == 0:
                    # If no objects detected
                    self.get_logger().info(f"No objects of the '{self.prompt}' prompt detected in image.")
                    # if object is not detected in frame, all target values will be -1 (see above)

                else:
                    # If at least one object detected

                    # Print calculated data to console
                    print("-------------------------------------------------------------------------------------------------")
                    for i, (cent_coord, avg_depth, phr, log) in enumerate(zip(centroids_as_pixels, avg_depths, phrases, logits)):
                        # Print info
                        self.get_logger().info(f"Mask {i+1}: {phr} (confidence of {log}")
                        self.get_logger().info(f"Centroid at: {cent_coord}, Average depth value: {avg_depth} mm")
                        print("-------------------------------------------------------------------------------------------------")

                process_dif = self.get_clock().now() - process_time_start
                total_seconds = process_dif.nanoseconds / 1e9
                log_string = str(f"Total processing time: {total_seconds:.9f} s")
                self.get_logger().info(log_string)
                print("-------------------------------------------------------------------------------------------------")

        except (requests.exceptions.RequestException, IOError) as e:
            self.get_logger().error(f"Error: {e}")

    def image_callback(self, rgb_msg, depth_msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        self.depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")

        depth_array = np.array(self.depth_image, dtype=np.float32)

        # self.get_logger().info("Image received!")

        if not REGULATE_PROCESS_RATE:
            self.process_images()
    
    def cam_info_callback(self, cam_info_msg:CameraInfo):
        self.camera_params.height = cam_info_msg.height
        self.camera_params.width = cam_info_msg.width
        instrinsic_matrix = cam_info_msg.p
        self.camera_params.fx = instrinsic_matrix[0][0]
        self.camera_params.fy = instrinsic_matrix[1][1]
        self.camera_params.cx = instrinsic_matrix[0][2]
        self.camera_params.cy = instrinsic_matrix[1][2]

        self._subscriptions.remove(self.cam_info_sub)

    def cleanup(self):
        self.get_logger().info("Shutting down Image Processor Node")

def main(args=None):
    rclpy.init(args=args)
    image_processor = ImageProcessor()
    try:
        rclpy.spin(image_processor)
    except KeyboardInterrupt:
        pass
    finally:
        image_processor.cleanup()
        image_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
