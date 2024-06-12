#include "wmm_control/HelperFunctions.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <thread>
#include <Eigen/Dense>
#include <ros/time.h>
#include <csignal>
#include <unistd.h>
#include <chrono>
#include <cmath>

using namespace Eigen;
using namespace std;

#define HOME_ACTION_IDENTIFIER 2
#define pi 3.1416

// Global constant definitions

// for PID controllers TODO adjust all these values
// Angular control
const double KP_ANGULAR = 0.1;
const double KI_ANGULAR = 0.01;
const double KD_ANGULAR = 0.05;
// Motor limits for angular velocity
const double MAX_ANG_VEL = 0.5;  // rad/s
const double MIN_ANG_VEL = -0.5; // rad/s

// Distance control
const double KP_DISTANCE = 0.1;
const double KI_DISTANCE = 0.01;
const double KD_DISTANCE = 0.05;
// Motor limits for linear velocity
const double MAX_LIN_VEL = 0.3;  // m/s
const double MIN_LIN_VEL = -0.3; // m/s

// Camera parameters - from: https://www.researchgate.net/publication/283482095_Calibration_of_Kinect_for_Xbox_One_and_Comparison_between_the_Two_Generations_of_Microsoft_Sensors
const int FRAME_WITDH = 1920;
const double HORIZONTAL_FOV_DEG = 70;
const double HORIZONTAL_FOV_RAD = HORIZONTAL_FOV_DEG * M_PI / 180.0;

// Target parameters
const int DESIRED_X = FRAME_WITDH / 2;                          // Target is the center of the frame
const int ACCEPTED_ERROR_RANGE_PX = FRAME_WITDH * 0.10 / 2; // 10% of the frame width

// Desired distance and error threshold
const double DESIRED_DIST = 1.0;         // meters
const double ACCEPTED_DIST_ERROR = 0.30; // meters

const double MAX_WHEEL_SPEED = 5; // Maximum wheel speed in rad/s // TODO adjust

// Define thresholds for force and torque on end effector
const double FORCE_TRESHOLD = 5;
const double TORQUE_TRESHOLD = 2.5;

// Global variable definitions
double end_f[3], end_tq[3];
double pos_base[4];
geometry_msgs::Point target_pos; // Holds the position of the target in x(px), y(px), z(mm)
bool update_target = false;

int loop_count = 0;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "wmm_control_node");
    ros::NodeHandle n("~");

    signal(SIGINT, signalHandler);

    // Set loop rate
    ros::Rate loop_rate(40);

    // Setup variables for publishing wheel and robot velocities
    std_msgs::Float64 velKF1, velKF2, velKR1, velKR2;
    geometry_msgs::Twist robot_vel;

    // Define publishers
    ros::Publisher velocityKF1_pub = n.advertise<std_msgs::Float64>("/velocityKF1_send", 1000);
    ros::Publisher velocityKF2_pub = n.advertise<std_msgs::Float64>("/velocityKF2_send", 1000);
    ros::Publisher velocityKR1_pub = n.advertise<std_msgs::Float64>("/velocityKR1_send", 1000);
    ros::Publisher velocityKR2_pub = n.advertise<std_msgs::Float64>("/velocityKR2_send", 1000);
    ros::Publisher base_velocity_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

    // Define subscribers
    ros::Subscriber sub1 = n.subscribe("/positionKF1_receive", 1000, &posKF1_rec);
    ros::Subscriber sub2 = n.subscribe("/positionKF2_receive", 1000, &posKF2_rec);
    ros::Subscriber sub3 = n.subscribe("/positionKR1_receive", 1000, &posKR1_rec);
    ros::Subscriber sub4 = n.subscribe("/positionKR2_receive", 1000, &posKR2_rec);
    ros::Subscriber sub5 = n.subscribe("/netft_data", 1000, &ft_rec);
    ros::Subscriber subTar = n.subscribe("/process/target", 1000, &target_pos_rec);

    // Log start time (in seconds)
    double time_start = ros::Time::now().toSec();

    // Declare velocity command variables
    float Vx_final = 0;
    float Vy_final = 0;
    float omega_final = 0;

    // Define PID controllers for object tracking
    PIDController pid_angular(KP_ANGULAR, KI_ANGULAR, KD_ANGULAR, MAX_ANG_VEL, MIN_ANG_VEL);
    PIDController pid_distance(KP_DISTANCE, KI_DISTANCE, KD_DISTANCE, MAX_LIN_VEL, MIN_LIN_VEL);
    auto previous_time = std::chrono::high_resolution_clock::now();

    // Main loop
    while (ros::ok())
    {
        // ------------------------------------------------------- Robot Control Loop ----------------------------------------------------------------
        /*
        Inputs available:
        -> target_pos - Point data type that stores target position data; this is split into the 3 variables seen below
        -> target_x - horizontal location of centroid of target object in pixel coordinates (from the perspective of the Kinect camera)
        -> target_y - vertical location of centroid of target object in pixel coordinates (from the perspective of the Kinect camera)
            NOTE: If the target object is not found in the image, both targetX and targetY will be -1
        -> target_depth - depth of centroid of target object (in mm)
            NOTE: If the target object is <0.5m or >4.5m from the camera, the depth registers as 0.0
                    If the object depth cannot be found or calculated, it returns -1

        Algorithm Description:
        This algorithm currently uses two simple PID controllers to
        1. turn the robot towards the target object, and
        2. maintain a specified distance from the target object.

        If no target object is detected, it will not move.
        If a target object is detected by the image recognition, but its centroid is not in the detection
        range of the depth camera, the robot will track the object by turning in place (no linear movement).
        */

        // Only calculate new speed values when a new target position is received
        if(update_target)
        {
            // Initialize base speeds
            double Vx = 0.0;     // forward speed
            double Vy = 0.0;     // sideways speed (strafe)
            double omega = 0.0;  // rotation speed

            // Get the current time (for PID)
            auto current_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = current_time - previous_time;
            double dt = elapsed.count();

            // Separate target position into 3 variables
            float target_x = target_pos.x;     // in px
            float target_y = target_pos.y;     // in px
            float target_depth = target_pos.z; // in mm
            std::cout << "Target position: (" << target_x << ", " << target_y << ", " << target_depth << ")" << std::endl;

            // Check if valid target
            if(target_x == -1 && target_y == -1)
            {
                // Target not in frame
                std::cout << "Target not found." << std::endl;
            }
            else
            {
                // Target in frame
                if (target_depth != 0 && target_depth != -1)
                {
                    // Target in detectable range
                    
                    // Drive towards target
                    // Calculate distance error
                    double distance_error = target_depth - DESIRED_DIST;
                    // Check if distance error is within the accepted range
                    if (std::abs(distance_error) > ACCEPTED_DIST_ERROR)
                    {
                        // Compute control output using PID controller for linear velocity
                        Vx = pid_distance.compute(distance_error, dt);
                    }
                    else
                    {
                        std::string ros_msg = "Distance within accepted range. No distance correction needed.";
                        ROS_INFO("%s", ros_msg.c_str());
                    }
                }
                else
                {
                    // Target in frame but >4.5m or <0.5m away
                    std::cout << "Target outside of detectable range." << std::endl;
                }

                // Turn towards target
                // Calculate pixel error
                double pixel_error = target_x - DESIRED_X;

                // Convert pixel error to angular error
                double angular_error = (pixel_error / FRAME_WITDH) * HORIZONTAL_FOV_RAD;

                // Calculate error range in radians
                double accepted_error_range_radians = (ACCEPTED_ERROR_RANGE_PX / FRAME_WITDH) * HORIZONTAL_FOV_RAD;

                // Check if angular error is within the accepted range
                if (std::abs(angular_error) > accepted_error_range_radians)
                {
                    // Compute control output using PID controller for angular velocity
                    omega = pid_angular.compute(angular_error, dt);
                }
                else
                {
                    std::string ros_msg = "Centroid within accepted range. No angular correction needed.";
                    ROS_INFO("%s", ros_msg.c_str());
                }
            }

            // Calculate individual wheel speeds from intended base speed
            VectorXd velocities = getWheelVelocities(Vx, Vy, omega); // calculate individual wheel velocities

            VectorXd wheel_speeds = velocities * 109 / pi;
            double max_speed = wheel_speeds.cwiseAbs().maxCoeff();

            // Scale wheel speeds if the maximum exceeds the allowed limit
            if (max_speed > MAX_WHEEL_SPEED)
                wheel_speeds = wheel_speeds * (MAX_WHEEL_SPEED / max_speed);
            
            std::cout << "Wheel speeds: " << wheel_speeds.transpose() << std::endl;

            // calculate real Vx, Vy, and omega from wheel speeds (after scaling)
            robot_vel = getRobotVelocities(wheel_speeds(0), wheel_speeds(1), wheel_speeds(2), wheel_speeds(3));

            std::cout << "Robot velocity: " << robot_vel << std::endl;

            // Assign messages for publishing
            velKF1.data = -wheel_speeds(1); // flip because of motor orientation
            velKF2.data = -wheel_speeds(0);
            velKR1.data = wheel_speeds(2);
            velKR2.data = wheel_speeds(3);

            // Publish wheel speeds
            velocityKF1_pub.publish(velKF1);
            velocityKF2_pub.publish(velKF2);
            velocityKR1_pub.publish(velKR1);
            velocityKR2_pub.publish(velKR2);

            // Publish robot velocities
            base_velocity_pub.publish(robot_vel);

            // Update previous time
            previous_time = current_time;

            // Set update to false, so that the loop doesn't run again until the target position is updated
            update_target = false;
            
            
            // -------------------------------------------------------------------------------------------------------------------------------------------
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}