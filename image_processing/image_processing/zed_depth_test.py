# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class DepthProcessing(Node):

    def __init__(self):
        super().__init__('zed_depth')
        self.bridge = CvBridge()
        
        # Declare the subscription
        self.subscription = self.create_subscription(
            Image,
            'zed/zed_node/depth/depth_registered',
            self.callback,
            10
        )

    def callback(self, depth_data):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_data, "32FC1")
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridgeError: {e}')

        depth_array = np.array(depth_image, dtype=np.float32)

        self.get_logger().info(f'Image size: {depth_data.width}x{depth_data.height}')

        u = depth_data.width // 2
        v = depth_data.height // 2

        self.get_logger().info(f'Center depth: {depth_array[v, u]:.2f} m')

def main(args=None):
    rclpy.init(args=args)
    depth_processing = DepthProcessing()
    rclpy.spin(depth_processing)
    depth_processing.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
