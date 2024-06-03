#!/usr/bin/env python3
# Authored by: Andor Siegers
# Assumes: 640x480 image from webcam

import cv2
import mediapipe as mp
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32

# -------------------------------------------- User defined constants ------------------------------------------------

# Select whether to display the output of the program or whether to simply publish the calculated angles to a ROS topic
DISPLAY_OUTPUT = True

# Change how often the pose is assessed
ASSESSMENT_RATE = 20 # in Hz

# --------------------------------------------------------------------------------------------------------------------

# -------------------------------------------- Calculated constants --------------------------------------------------

# Calculate the display interval in seconds
ASSESSMENT_INTERVAL = 1.0 / ASSESSMENT_RATE # in seconds

# --------------------------------------------------------------------------------------------------------------------

# Function to calculate angle between three points
def calculate_angle(a, b, c):
    a = np.array(a)  # First point
    b = np.array(b)  # Second point (vertex)
    c = np.array(c)  # Third point

    radians = np.arctan2(c[1] - b[1], c[0] - b[0]) - np.arctan2(a[1] - b[1], a[0] - b[0])
    angle = np.abs(radians * 180.0 / np.pi)

    if angle > 180.0:
        angle = 360 - angle

    return angle
class PoseAssess:
    def __init__(self):
        self.bridge = CvBridge()

        # Initialize node
        rospy.init_node('assess_pose', anonymous=True)

        # Initialize MediaPipe pose
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose()

        # Initialize MediaPipe drawing
        self.mp_drawing = mp.solutions.drawing_utils

        # Initialize USB frame
        self.usb_frame = None

        # Subscriber to update usb image everytime one is received
        self.usb_cam_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.callback)

        # Timer to assess pose
        self.assess_timer = rospy.Timer(rospy.Duration(ASSESSMENT_INTERVAL), self.assess_pose)

        # Publishers to publish pose angles
        self.left_arm_ang_pub = rospy.Publisher('/pose_assess/left_arm_ang', Float32, queue_size=10)
        self.right_arm_ang_pub = rospy.Publisher('/pose_assess/right_arm_ang', Float32, queue_size=10)

        rospy.loginfo("Pose Assessment Node Started")

    def callback(self, usb_img_msg):
        try:
            # Convert the usb rgb image to OpenCV format
            self.usb_frame = self.bridge.imgmsg_to_cv2(usb_img_msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"Failed to convert images: {e}")
        except Exception as e:
            rospy.logerr(f"Error in callback_no_mask: {e}")

    def assess_pose(self, event):
        if self.usb_frame is None:
            return

        # Assess pose
        # Convert the frame to RGB
        image = cv2.cvtColor(self.usb_frame, cv2.COLOR_BGR2RGB)

        # Process the frame with MediaPipe Pose
        results = self.pose.process(image)

        # Draw pose landmarks
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if results.pose_landmarks:
            self.mp_drawing.draw_landmarks(image, results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS)

            # Extract landmarks
            landmarks = results.pose_landmarks.landmark

            # Get coordinates of shoulder, elbow, and wrist for left and right arms
            left_shoulder = [landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER.value].x, landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER.value].y]
            left_elbow = [landmarks[self.mp_pose.PoseLandmark.LEFT_ELBOW.value].x, landmarks[self.mp_pose.PoseLandmark.LEFT_ELBOW.value].y]
            left_wrist = [landmarks[self.mp_pose.PoseLandmark.LEFT_WRIST.value].x, landmarks[self.mp_pose.PoseLandmark.LEFT_WRIST.value].y]

            right_shoulder = [landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER.value].x, landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER.value].y]
            right_elbow = [landmarks[self.mp_pose.PoseLandmark.RIGHT_ELBOW.value].x, landmarks[self.mp_pose.PoseLandmark.RIGHT_ELBOW.value].y]
            right_wrist = [landmarks[self.mp_pose.PoseLandmark.RIGHT_WRIST.value].x, landmarks[self.mp_pose.PoseLandmark.RIGHT_WRIST.value].y]

            # Calculate angles
            left_angle = calculate_angle(left_shoulder, left_elbow, left_wrist)
            right_angle = calculate_angle(right_shoulder, right_elbow, right_wrist)

            # Publish angles
            self.left_arm_ang_pub.publish(left_angle)
            self.right_arm_ang_pub.publish(right_angle)

            
            if DISPLAY_OUTPUT:
                # Display angle values on the image
                cv2.putText(image, str(int(left_angle)),
                            tuple(np.multiply(left_elbow, [640, 480]).astype(int)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2, cv2.LINE_AA)

                cv2.putText(image, str(int(right_angle)),
                            tuple(np.multiply(right_elbow, [640, 480]).astype(int)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2, cv2.LINE_AA)

        # Display the frame
        if DISPLAY_OUTPUT:
            cv2.imshow('Mediapipe Feed', image)
            cv2.waitKey(1)

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        pose_assess = PoseAssess()
        pose_assess.spin()
    except rospy.ROSInterruptException:
        pass