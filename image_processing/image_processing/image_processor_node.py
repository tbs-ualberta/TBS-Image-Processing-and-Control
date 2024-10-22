# Authored by: Andor Siegers

# DO NOT INSTALL IN CONDA ENVIRONMENT: this caused many incompatibility issues when tested

import rclpy
import rclpy.logging
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
import numpy as np
sys.path.append(os.path.join(os.path.dirname(__file__)))
from utils import CameraParams, calculate_centroids, get_avg_depth, find_target, convert_to_MaskArray, get_point_xyz
import warnings
from transformers import logging
from rclpy.duration import Duration

# Suppress unimportant messages printing to the console
warnings.filterwarnings("ignore", category=UserWarning)
warnings.filterwarnings("ignore", message="torch.meshgrid: in an upcoming release")
logging.set_verbosity_error()

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        # Initialize CvBridge
        self.bridge = CvBridge()

        # -------------------------------------------- ROS Parameters ------------------------------------------------
        
        # Specifies the prompt which the model masks
        self.prompt = self.declare_parameter('prompt', 'person').get_parameter_value().string_value # multiple objects can be detected using a '.' as a separator

        # Specifies target. This allows for detection of multiple objects but only targetting of one.
        self.target = self.declare_parameter('target', 'person').get_parameter_value().string_value # must be contained in the prompt
        
        # The minimum confidence the model must have to consider the object as a target
        self.target_confidence_threshold = self.declare_parameter('target_confidence_threshold', 0.5).get_parameter_value().double_value

        # Specifies whether to print the output to the terminal or not
        self.print_output_bool = self.declare_parameter('print_output', False).get_parameter_value().string_value

        # Specifies whether the output should clear the console before outputting each cycle data
        self.clear_output_bool = self.declare_parameter('clear_output', True).get_parameter_value().bool_value

        # Specifies whether the processing rate should be regulated, or whether it should process as fast as possible
        self.regulate_process_rate_bool = self.declare_parameter('regulate_process_rate', False).get_parameter_value().bool_value

        # Processing frequency (in Hz): this is the max frequency. It should be noted that each cycle will take at 
        # minimum the amount of time needed to process one image, regardless of the set rate.
        self.processing_rate = self.declare_parameter('processing_rate', 1.0).get_parameter_value().double_value
        if self.processing_rate == 0: # if processing rate is 0, don't regulate
            self.regulate_process_rate_bool = False
        else:
            self.processing_interval = 1.0 / self.processing_rate # in seconds

        # Specifies RGB image topic name
        rgb_image_topic = self.declare_parameter('rgb_image_topic', 'zed/zed_node/rgb/image_rect_color').get_parameter_value().string_value # NOTE: this uses the rectified image, which may not be correct. It needs to be validated

        # Specifies Depth image topic name
        depth_img_topic = self.declare_parameter('depth_img_topic', 'zed/zed_node/depth/depth_registered').get_parameter_value().string_value

        # Specifies the camera info topic name
        cam_info_topic = self.declare_parameter('cam_info_topic', 'zed/zed_node/depth/camera_info').get_parameter_value().string_value

        # Debug: print the parameters to verify
        self.get_logger().info(f'prompt: {self.prompt}')
        self.get_logger().info(f'target: {self.target}')
        self.get_logger().info(f'print_output: {self.print_output_bool}')
        self.get_logger().info(f'clear_output: {self.clear_output_bool}')
        self.get_logger().info(f'rgb image topic: {rgb_image_topic}')
        self.get_logger().info(f'depth image topic: {depth_img_topic}')

        # ------------------------------------------------------------------------------------------------------------
        
        # Define publisher for mask data
        self.mask_pub = self.create_publisher(MaskArray, "process/mask_data", 10)

        # Define publisher for target position data
        self.target_pub = self.create_publisher(Point, "process/target", 10) # in px

        # Define subscriber for camera info. This is called only once and then is destroyed, as the info it receives does not change throughout the program runtime.
        self.cam_info_sub = self.create_subscription(CameraInfo, cam_info_topic, self.cam_info_callback, 10)

        # Define subscribers for camera image data (i.e. rgb and depth images)
        self.left_rgb_sub = message_filters.Subscriber(self, Image, rgb_image_topic)
        self.left_depth_reg_sub = message_filters.Subscriber(self, Image, depth_img_topic)
        self.ts = message_filters.TimeSynchronizer([self.left_rgb_sub, self.left_depth_reg_sub], queue_size=10)
        self.ts.registerCallback(self.image_callback)

        self.camera_params = CameraParams()
        self.rgb_image = None
        self.depth_image = None

        # Initialize model
        self.model = LangSAM(sam_type = "vit_b")

        if self.regulate_process_rate_bool:
            # Set up timers to call process function (only if process rate is being regulated)
            self.processing_timer = self.create_timer(self.processing_interval, self.process_images)
        
        self.get_logger().info("Image Processor Node Started")

    def process_images(self):
        if self.rgb_image is None or self.depth_image is None:
            self.get_logger().info("Depth image and/or rgb image not received. Waiting to process...", throttle_duration_sec=3)
            return
        elif self.camera_params.width == 0:
            self.get_logger().info("Camera parameters not received. Waiting to process...", throttle_duration_sec=3)
            return
        
        try:
            # Save start time (to evaluate process time)
            process_time_start = self.get_clock().now()

            rgb_image = self.rgb_image
            depth_image = self.depth_image

            self.get_logger().info("RGB image size: " + str(rgb_image.shape))
            self.get_logger().info("Depth image size: " + str(depth_image.shape))

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

            # Find each centroid's point in 3D cartesian space, with the depth camera as the origin
            centroids_cartesian = []
            for centroid_as_px in centroids_as_pixels:
                centroids_cartesian.append(get_point_xyz(centroid_as_px, depth_array, self.camera_params))
            print("Cartesian Centroids: " + str(centroids_cartesian))

            # Find average depth value of each mask
            avg_depths = []
            avg_depths = [get_avg_depth(mask, depth_array) for mask in masks_np]
            print("Average Depths: " + str(avg_depths))

            # Find target
            target_pos = find_target(self.target, self.target_confidence_threshold, centroids_cartesian, phrases, avg_depths, logits)
            print("Found target at: " + str(target_pos))

            # Convert to correct message type for publishing
            mask_array = convert_to_MaskArray(centroids_as_pixels, centroids_cartesian, phrases, logits, avg_depths, masks_np, rgb_image, depth_image)
            
            # Publish target values
            self.target_pub.publish(target_pos)

            # Publish mask data
            self.mask_pub.publish(mask_array)

            if self.print_output_bool:
                # Clear the console for new output
                if self.clear_output_bool:
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
        # self.get_logger().info("Image received!")
        if not self.regulate_process_rate_bool:
            self.process_images()
    
    def cam_info_callback(self, cam_info_msg:CameraInfo):
        # Get camera info from standardized camera info message (http://docs.ros.org/en/diamondback/api/sensor_msgs/html/__CameraInfo_8py_source.html)
        try:
            self.camera_params.height = cam_info_msg.height
            self.camera_params.width = cam_info_msg.width
            instrinsic_matrix = cam_info_msg.p
            self.camera_params.fx = instrinsic_matrix[0]
            self.camera_params.fy = instrinsic_matrix[5]
            self.camera_params.cx = instrinsic_matrix[2]
            self.camera_params.cy = instrinsic_matrix[6]
            
            self.get_logger().info("Camera info retrieved. Using following values:")
            self.get_logger().info("Height: " + str(self.camera_params.height))
            self.get_logger().info("Width: " + str(self.camera_params.width))
            self.get_logger().info("fx: " + str(self.camera_params.fx))
            self.get_logger().info("fy: " + str(self.camera_params.fy))
            self.get_logger().info("cx: " + str(self.camera_params.cx))
            self.get_logger().info("cy: " + str(self.camera_params.cy))

            self._subscriptions.remove(self.cam_info_sub)
        except IndexError as e:
            self.get_logger().error("Unable to save camera info: " + str(e) +"\nRetrying...")

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
