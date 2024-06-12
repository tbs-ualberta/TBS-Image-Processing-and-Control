// Andor Siegers and Mahdi Chalaki - based off an example provided by Kinova for their gen3 arm
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
const double KI_ANGULAR = 0.00;
const double KD_ANGULAR = 0.05;
// Motor limits for angular velocity
const double MAX_ANG_VEL = 5.0;  // rad/s  // 0.5
const double MIN_ANG_VEL = -5.0; // rad/s // -0.5

// Distance control
const double KP_DISTANCE = 0.1;
const double KI_DISTANCE = 0.00;
const double KD_DISTANCE = 0.05;
// Motor limits for linear velocity
const double MAX_LIN_VEL = 0.5; // m/s // should be 0.3
const double MIN_LIN_VEL = 0.0; // m/s // Robot will not go backwards

// Camera parameters - from: https://www.researchgate.net/publication/283482095_Calibration_of_Kinect_for_Xbox_One_and_Comparison_between_the_Two_Generations_of_Microsoft_Sensors
const int FRAME_WITDH = 1920;
const double HORIZONTAL_FOV_DEG = 70;
const double HORIZONTAL_FOV_RAD = HORIZONTAL_FOV_DEG * M_PI / 180.0;

// Target parameters
const int DESIRED_X = FRAME_WITDH / 2;                      // Target is the center of the frame
const int ACCEPTED_ERROR_RANGE_PX = FRAME_WITDH * 0.30 / 2; // 30% of the frame width

// Desired distance and error threshold
const double DESIRED_DIST = 1.0;         // meters
const double ACCEPTED_DIST_ERROR = 0.50; // meters // TODO if approaching robot it shouldn't go backwards

const double MAX_WHEEL_SPEED = 100; // 45 // Maximum wheel speed in rad/s // This may be unnecessary

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

    // Kinova arm Parameters
    string robot_name = "my_gen3";
    int degrees_of_freedom = 7;
    bool is_gripper_present = false;

    // Get Kinova arm information and set to defaults
    get_arm_info(&robot_name, &degrees_of_freedom, &is_gripper_present);

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

    // Make sure to clear the robot's faults else it won't move if it's already in fault
    clear_faults(n, robot_name);

    // Move arm to start position
    VectorXd start_position(7, 1);
    start_position << 0, 0, 0, 0, 0, 0, 90; // arm straight upwards
    send_joint_angles(n, robot_name, degrees_of_freedom, start_position);
    // std::this_thread::sleep_for(std::chrono::milliseconds(10000));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000)); // for testing

    // Move arm to walker position
    VectorXd walker_position(7, 1);
    walker_position << 0, 25.3, 0, 278, 0, 328.56, 90; // Experimental values - Handles in good position
    send_joint_angles(n, robot_name, degrees_of_freedom, walker_position);
    // std::this_thread::sleep_for(std::chrono::milliseconds(10000));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000)); // for testing

    // Define a vector for joint speeds
    VectorXd kortex_speed(7, 1);

    // Log start time (in seconds)
    double time_start = ros::Time::now().toSec();

    // Setup file for force logging
    // FILE *fpforce;
    // fpforce = fopen("/home/user/Desktop/force.txt", "w");
    // fprintf(fpforce, "t   fx   fy    fz  tq_x   tq_y   tq_z   f_vertical   f_front   tau\r\n");

    // Declare velocity command variables
    double time_old = 0;
    float Vx_final = 0;
    float Vy_final = 0;
    float omega_final = 0;

    // // For removing offset
    // int window_size = 100;
    // MatrixXd biased_data_history(6, window_size);
    // VectorXd biased_data_points(6);
    // VectorXd row_means(6);

    // Define PID controllers for object tracking
    PIDController pid_angular(KP_ANGULAR, KI_ANGULAR, KD_ANGULAR, MAX_ANG_VEL, MIN_ANG_VEL);
    PIDController pid_distance(KP_DISTANCE, KI_DISTANCE, KD_DISTANCE, MAX_LIN_VEL, MIN_LIN_VEL);
    auto previous_time = std::chrono::high_resolution_clock::now();

    // Main loop
    while (ros::ok())
    {
        // For keeping track of time (in seconds)
        double time_now = ros::Time::now().toSec();
        double time1 = time_now - time_start;

        // // To remove offset from force sensor
        // if (loop_count < 100)
        // {
        //     // adds force data to matrix to be used in calibration
        //     biased_data_points << end_f[0], end_f[1], end_f[2], end_tq[0], end_tq[1], end_tq[2];
        //     biased_data_history.col(loop_count) = biased_data_points;
        // }

        // Calculate mean of each row
        // row_means = biased_data_history.rowwise().mean();

        if ((time1 > 4.0 && time1 < 6.0))
        {
            // Set arm to static position
            kortex_speed[0] = 0 / 10.0; // TODO why divide by 10.0?
            kortex_speed[1] = 0 / 10.0;
            kortex_speed[2] = 0 / 10.0;
            kortex_speed[3] = 0 / 10.0;
            kortex_speed[4] = 0 / 10.0;
            kortex_speed[5] = 0 / 10.0;
            kortex_speed[6] = 0 / 10.0;
            send_joint_speeds(n, robot_name, degrees_of_freedom, kortex_speed);

            // Move forward slightly, indicating robot is ready
            double Vx = 0.01;   // forward speed
            double Vy = 0.0;    // sideways speed (strafe)
            double omega = 0.0; // rotation speed
            VectorXd velocities = getWheelVelocities(Vx, Vy, omega);
            VectorXd wheel_speeds = velocities * 109 / pi;

            // Assign messages for publishing
            velKF1.data = -wheel_speeds(1);
            velKF2.data = -wheel_speeds(0);
            velKR1.data = wheel_speeds(2);
            velKR2.data = wheel_speeds(3);

            // Publish wheel speeds
            velocityKF1_pub.publish(velKF1);
            velocityKF2_pub.publish(velKF2);
            velocityKR1_pub.publish(velKR1);
            velocityKR2_pub.publish(velKR2);
        }
        else if (time1 > 6.0 || true) // the true term in this if statement should be removed when force sensing needs to be added back in
        {
            // Regular operation begins
            
            // Set all arm speeds to 0 to keep it in the same position
            kortex_speed[0] = 0 / 10.0;
            kortex_speed[1] = 0 / 10.0;
            kortex_speed[2] = 0 / 10.0;
            kortex_speed[3] = 0 / 10.0;
            kortex_speed[4] = 0 / 10.0;
            kortex_speed[5] = 0 / 10.0;
            kortex_speed[6] = 0 / 10.0;
            send_joint_speeds(n, robot_name, degrees_of_freedom, kortex_speed);

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
            If the target object comes closer to the robot, the robot will hold it's position (will not backup).
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
                target_depth = target_depth / 1000;// convert to meters

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
                    if (target_depth > 0.0)
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
                        std::string ros_msg = "Target outside of detectable range.";
                        ROS_INFO("%s", ros_msg.c_str());
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
                VectorXd velocities = getWheelVelocities(Vx, Vy, omega); // Calculate individual wheel velocities

                VectorXd wheel_speeds = velocities * 109 / pi;
                double max_speed = wheel_speeds.cwiseAbs().maxCoeff();

                // Scale wheel speeds if the maximum exceeds the allowed limit
                if (max_speed > MAX_WHEEL_SPEED)
                    wheel_speeds = wheel_speeds * (MAX_WHEEL_SPEED / max_speed);
                
                std::cout << "Wheel speeds: " << wheel_speeds.transpose() << std::endl;

                // Calculate real Vx, Vy, and omega from wheel speeds (after scaling)
                robot_vel = getRobotVelocities(wheel_speeds(0), wheel_speeds(1), wheel_speeds(2), wheel_speeds(3));

                std::cout << "Robot velocity: " << std::endl << robot_vel << std::endl;

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
            }
            
            // -------------------------------------------------------------------------------------------------------------------------------------------

            /*
            // Old control function - controls robot based on the forces exerted by the user on the handles attached to the arm

            // initialize base speeds
            double Vx_final = 0.0;    // forward speed
            double Vy_final = 0.0;    // sideways speed (strafe)
            double omega_final = 0.0; // rotation speed

            // process forces (apply offset and compensate for geometry of setup, force sensor mounted on an angle)
            double f_vertical = -0.7071 * ((end_f[0] - row_means(0)) - (end_f[1] - row_means(1))); // x-axis is in 45 and y-axis is in 135 from ground normal - 0.7071 is cos(45)
            double fz = end_f[2] - row_means(2);
            double tau_y = end_tq[1] - row_means(4);

            if (fz < -FORCE_TRESHOLD) // fz < -5
            {
                // move forward
                Vx = 0.30;
                Vy = 0.0;
                omega = 0.0;

                // turn robot if torque is sufficient
                // this will result in the robot driving through a curve, as the robot is also moving forward
                if (tau_y > TORQUE_TRESHOLD) // tau_y > 2.5
                {
                    omega = -0.3;
                }
                else if (tau_y < -TORQUE_TRESHOLD) // tau_y < -2.5
                {
                    omega = 0.3;
                }
            }
            else if (fz > FORCE_TRESHOLD) // fz > 5
            {
                // move backwards
                Vx = -0.30;
                Vy = 0.0;
                omega = 0.0;
            }
            else
            {
                // if force is below threshold, and torque is above threshold, turn robot in place
                if (tau_y > TORQUE_TRESHOLD) // tau_y > 2.5
                {
                    Vx = 0.0;
                    // Vy = 0.0;
                    omega = -0.5;
                }
                else if (tau_y < -TORQUE_TRESHOLD) // tau_y < -2.5
                {
                    Vx = 0.0;
                    // Vy = 0.0;
                    omega = 0.5;
                }
            }

            // uses a rolling function to calculate final speed
            // this reduces jitter in the system that comes from sudden changes in force
            float ratio = 0.05; // determines what percent of the final speed is dictated by the new velocity/omega value
            Vx_final = (1 - ratio) * Vx_final + ratio * Vx;
            Vy_final = (1 - ratio) * Vy_final + ratio * Vy;
            omega_final = (1 - ratio) * omega_final + ratio * omega;

            VectorXd velocities = getWheelVelocities(Vx_final, Vy_final, omega_final); // calculate individual wheel velocities
            VectorXd wheel_speeds = velocities * 109 / pi;

            // assign messages for publishing
            velKF1.data = -wheel_speeds(1);
            velKF2.data = -wheel_speeds(0);
            velKR1.data = wheel_speeds(2);
            velKR2.data = wheel_speeds(3);

            // publish wheel speeds
            velocityKF1_pub.publish(velKF1);
            velocityKF2_pub.publish(velKF2);
            velocityKR1_pub.publish(velKR1);
            velocityKR2_pub.publish(velKR2);

            // print raw force values, along with processed force values
            if (loop_count % 20 == 0)
            {
                cout << end_f[0] << "  " << end_f[1] << "  " << end_f[2] << "  " << end_tq[0] << "  " << end_tq[1] << "  " << end_tq[2] << " " << f_vertical << " " << fz << " " << tau_y << endl;
            }

            loop_count = loop_count + 1;

            // logs all raw and processed force values to a file
            fprintf(fpforce, "%.5f   %.5f   %.5f    %.5f    %.5f    %.5f   %.5f   %.5f   %.5f   %.5f\r\n", time1, end_f[0], end_f[1], end_f[2], end_tq[0], end_tq[1], end_tq[2], f_vertical, fz, tau_y);

            */
        }

        time_old = time_now;

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
} // end main

// TODO write launch file to launch WMM
// Write documentation that explains launch file