# Authored by: Andor Siegers

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist

class RangerController(Node):
    def __init__(self):
        super().__init__('ranger_controller')

        # Define publishing topics
        self.cmd_vel_topic = self.declare_parameter('cmd_vel_topic', '/mux/cmd_vel').get_parameter_value().string_value
        self.target_topic = self.declare_parameter('target_topic', '/process/target').get_parameter_value().string_value

        # Define linear and angular velocities
        self.LIN_VEL = self.declare_parameter('lin_vel', '0.5').get_parameter_value().double_value # in m/s
        self.ANG_VEL = self.declare_parameter('ang_vel', '3.0').get_parameter_value().double_value # in rad/s

        # Define following distance (max distance from the target the robot can be without moving)
        self.FOLLOW_DIST = self.declare_parameter('follow_dist','0.7').get_parameter_value().double_value # in meters

        # Define angular tolerance (window in which target can be without the robot moving)
        self.ANG_TOL = self.declare_parameter('ang_tol', '1.0').get_parameter_value().double_value # in rad
        
        # Define publisher for cmd_vel
        self.cmd_vel_pub= self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.get_logger().info(f'Publishing velocity commands to: {self.cmd_vel_topic}')

        # Define subscriber for target data
        self.target_sub = self.create_subscription(Point, self.target_topic, self.callback_target, 10)
        
        self.get_logger().info("Ranger Control Node Started")

    def callback_target(self, target_msg):
        # Get location of target in relation to depth camera (in meters)
        tar_x = target_msg.x
        tar_y = target_msg.y
        tar_z = target_msg.z

        # Initiate robot velocity variable
        vel = Twist()

        # Define image width and window tolerance
        IMAGE_WIDTH = 1280
        ALLOWABLE_WINDOW = 300 # px

        if tar_x == -1:
            print("No valid target")
            vel.angular.z = 0.0
        elif tar_x < IMAGE_WIDTH/2 - ALLOWABLE_WINDOW/2:
            print("Turning right")
            vel.angular.z = 0.5
        elif tar_x > IMAGE_WIDTH/2 + ALLOWABLE_WINDOW/2:
            print("Turning left")
            vel.angular.z = -0.5
        else:
            print("Target within range")
            vel.angular.z = 0.0
        
        '''
        # Check if target was found in image
        if tar_x == -1 and tar_y == -1 and tar_z == -1:
            # If target not found in image
            vel.linear.x = 0
            vel.angular.z = 0
        else:
            # If target found in image

            # Find angle to target from optical axis (perpendicular to sensor)
            target_ang = math.atan(tar_x/tar_z) # y/x (in rad)

            # Find linear distance from camera to target in the horizontal plane
            target_dist = math.sqrt(tar_x**2 + tar_z**2) # in meters

            # Check if target is within angle tolerance
            if target_ang < -self.ANG_TOL/2:    # if target is too far right, rotate ccw
                vel.linear.x = self.ANG_VEL
            elif target_ang > self.ANG_TOL/2:   # if target is too far left, rotate cw
                vel.linear.x = -self.ANG_VEL
            
            # Check if target is within following distance
            if target_dist > self.FOLLOW_DIST:
                vel.angular.z = self.LIN_VEL
        '''

        # Publish velocity data
        self.cmd_vel_pub.publish(vel)

    def cleanup(self):
        self.get_logger().info("Shutting down Ranger Controller Node")

def main(args=None):
    rclpy.init(args=args)
    ranger_controller = RangerController()
    try:
        rclpy.spin(ranger_controller)
    except KeyboardInterrupt:
        pass
    finally:
        ranger_controller.cleanup()
        ranger_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()