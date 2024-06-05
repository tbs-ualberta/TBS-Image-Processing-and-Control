#!/usr/bin/env python3
# Authored by: Andor Siegers
# Assumes: 640x480 image from webcam

import cv2
import mediapipe as mp
import numpy as np
import math
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32
from sensor_msgs.msg import Image as MsgImg
import message_filters

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
def calculate_angle_3d(a, b, c):       

    v1 = np.array([ a[0] - b[0], a[1] - b[1], a[2] - b[2] ])
    v2 = np.array([ c[0] - b[0], c[1] - b[1], c[2] - b[2] ])

    v1mag = np.sqrt([ v1[0] * v1[0] + v1[1] * v1[1] + v1[2] * v1[2] ])
    v1norm = np.array([ v1[0] / v1mag, v1[1] / v1mag, v1[2] / v1mag ])

    v2mag = np.sqrt(v2[0] * v2[0] + v2[1] * v2[1] + v2[2] * v2[2])
    v2norm = np.array([ v2[0] / v2mag, v2[1] / v2mag, v2[2] / v2mag ])
    res = v1norm[0] * v2norm[0] + v1norm[1] * v2norm[1] + v1norm[2] * v2norm[2]
    angle_rad = np.arccos(res)

    return math.degrees(angle_rad)

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

        # Initialize images
        self.rgb_image = None
        self.depth_image = None

        # Define subscribers for depth and rgb topics
        self.rgb_sub = message_filters.Subscriber('/kinect2/rgb', MsgImg)
        self.depth_sub = message_filters.Subscriber('/kinect2/depth_raw', MsgImg)

        # Synchronize the topics
        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], 10, 1.0)
        self.ts.registerCallback(self.callback)

        # Timer to assess pose
        self.assess_timer = rospy.Timer(rospy.Duration(ASSESSMENT_INTERVAL), self.assess_pose)

        # Publishers to publish pose angles
        self.left_arm_ang_pub = rospy.Publisher('/pose_assess/left_arm_ang', Float32, queue_size=10)
        self.right_arm_ang_pub = rospy.Publisher('/pose_assess/right_arm_ang', Float32, queue_size=10)

        rospy.loginfo("Pose Assessment Node Started")

    def callback(self, rgb_msg, depth_msg):
        try:
            # Convert the rgb and depth images to OpenCV format
            self.rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            self.depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="32FC1")

        except CvBridgeError as e:
            rospy.logerr(f"Failed to convert images: {e}")

    def assess_pose(self, event):
        if self.usb_frame is None:
            return

        # Assess pose
        # Convert the frame to RGB
        image = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2RGB)

        # Process the frame with MediaPipe Pose
        results = self.pose.process(image)

        # Draw pose landmarks
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if results.pose_landmarks:
            self.mp_drawing.draw_landmarks(image, results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS)

            # Extract landmarks
            landmarks = results.pose_landmarks.landmark

            # TODO implement 3D angle detection
            # Get coordinates of shoulder, elbow, and wrist for left and right arms
            left_shoulder = [landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER.value].x, landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER.value].y, landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER.value].z]
            left_elbow = [landmarks[self.mp_pose.PoseLandmark.LEFT_ELBOW.value].x, landmarks[self.mp_pose.PoseLandmark.LEFT_ELBOW.value].y, landmarks[self.mp_pose.PoseLandmark.LEFT_ELBOW.value].z]
            left_wrist = [landmarks[self.mp_pose.PoseLandmark.LEFT_WRIST.value].x, landmarks[self.mp_pose.PoseLandmark.LEFT_WRIST.value].y, landmarks[self.mp_pose.PoseLandmark.LEFT_WRIST.value].z]

            right_shoulder = [landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER.value].x, landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER.value].y, landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER.value].z]
            right_elbow = [landmarks[self.mp_pose.PoseLandmark.RIGHT_ELBOW.value].x, landmarks[self.mp_pose.PoseLandmark.RIGHT_ELBOW.value].y,landmarks[self.mp_pose.PoseLandmark.RIGHT_ELBOW.value].z]
            right_wrist = [landmarks[self.mp_pose.PoseLandmark.RIGHT_WRIST.value].x, landmarks[self.mp_pose.PoseLandmark.RIGHT_WRIST.value].y,landmarks[self.mp_pose.PoseLandmark.RIGHT_WRIST.value].z]

            # Calculate angles
            left_angle = calculate_angle_3d(left_shoulder, left_elbow, left_wrist)
            right_angle = calculate_angle_3d(right_shoulder, right_elbow, right_wrist)

            # Publish angles (in degrees)
            self.left_arm_ang_pub.publish(left_angle)
            self.right_arm_ang_pub.publish(right_angle)

            
            if DISPLAY_OUTPUT:
                # Display angle values on the image
                cv2.putText(image, str(int(left_angle)),
                            tuple(np.multiply([left_elbow[0],left_elbow[1]], [640, 480]).astype(int)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2, cv2.LINE_AA)

                cv2.putText(image, str(int(right_angle)),
                            tuple(np.multiply([right_elbow[0],right_elbow[1]], [640, 480]).astype(int)),
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