#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from pynput import keyboard

# Initialize ROS node
rospy.init_node('teleop_wasd')

# Publishers for each wheel controller
pub_front_left = rospy.Publisher('/wheel_front_left_controller/command', Float64, queue_size=10)
pub_front_right = rospy.Publisher('/wheel_front_right_controller/command', Float64, queue_size=10)
pub_back_left = rospy.Publisher('/wheel_back_left_controller/command', Float64, queue_size=10)
pub_back_right = rospy.Publisher('/wheel_back_right_controller/command', Float64, queue_size=10)

# Speeds for the wheels
wheel_speed = 100.0

# Callback function to handle key press events
def on_press(key):
    try:
        if key.char == 'w':
            # Move forward
            pub_front_left.publish(wheel_speed)
            pub_front_right.publish(wheel_speed)
            pub_back_left.publish(wheel_speed)
            pub_back_right.publish(wheel_speed)
        elif key.char == 's':
            # Move backward
            pub_front_left.publish(-wheel_speed)
            pub_front_right.publish(-wheel_speed)
            pub_back_left.publish(-wheel_speed)
            pub_back_right.publish(-wheel_speed)
        elif key.char == 'a':
            # Turn left
            pub_front_left.publish(-wheel_speed)
            pub_front_right.publish(wheel_speed)
            pub_back_left.publish(wheel_speed)
            pub_back_right.publish(-wheel_speed)
        elif key.char == 'd':
            # Turn right
            pub_front_left.publish(wheel_speed)
            pub_front_right.publish(-wheel_speed)
            pub_back_left.publish(-wheel_speed)
            pub_back_right.publish(wheel_speed)
    except AttributeError:
        pass

# Callback function to handle key release events
def on_release(key):
    # Stop the robot when any key is released
    pub_front_left.publish(0)
    pub_front_right.publish(0)
    pub_back_left.publish(0)
    pub_back_right.publish(0)

# Main function
def main():
    # Create a keyboard listener
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    # Keep the script running
    rospy.spin()

if __name__ == '__main__':
    main()
