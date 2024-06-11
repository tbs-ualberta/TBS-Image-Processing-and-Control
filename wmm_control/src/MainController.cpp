
#include "wmm_control/HelperFunctions.h"

#include <thread>
#include "ros/ros.h"
#include <iostream>
#include <Eigen/Dense>
#include <ros/time.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Point.h>
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"

#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/TwistStamped.h"

#include <csignal>
#include <unistd.h>
#include <chrono>
#include <cmath>

using namespace Eigen;
using namespace std;

#define HOME_ACTION_IDENTIFIER 2
#define pi 3.1416

// global constant definitions
// for PID controllers
// Angular control
const double Kp_angular = 0.1;
const double Ki_angular = 0.01;
const double Kd_angular = 0.05;
// Motor limits for angular velocity
const double max_angular_velocity = 1.0;  // rad/s
const double min_angular_velocity = -1.0; // rad/s
// Distance control
const double Kp_distance = 0.1;
const double Ki_distance = 0.01;
const double Kd_distance = 0.05;
// Motor limits for linear velocity
const double max_linear_velocity = 0.5;  // m/s
const double min_linear_velocity = -0.5; // m/s

// Camera parameters
const int frame_width = 640;
const double horizontal_fov_degrees = 90.0;
const double horizontal_fov_radians = horizontal_fov_degrees * M_PI / 180.0;

// Target parameters
const int desired_x = frame_width / 2;                          // Target is the center of the frame
const int accepted_error_range_pixels = frame_width * 0.10 / 2; // 10% of the frame width

// Desired distance and error threshold
const double desired_distance = 3.0;         // meters
const double accepted_distance_error = 0.30; // meters

// Robot parameters
const double r = constants::RADIUS;      // Wheel radius in meters
const double d = hypot(constants::WIDTH/2, constants::LENGTH/2); // Distance from wheel to center of the robot

// define thresholds for force and torque on end effector
const double f_thresh = 5;
const double t_thresh = 2.5;

const double max_wheel_speed = 1.0; // Maximum wheel speed in rad/s

// global variable definitions
double end_f[3], end_tq[3];
double pos_base[4];
geometry_msgs::Point target_pos; // Defines the position of the target in x(px), y(px), z(mm)

int loop_count = 0;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "WMM_control_node");
    ros::NodeHandle n("~");

    // Kinova arm Parameters
    string robot_name = "my_gen3";
    int degrees_of_freedom = 7;
    bool is_gripper_present = false;

    get_arm_info(&robot_name, &degrees_of_freedom, &is_gripper_present);

    signal(SIGINT, signalHandler);

    // set loop rate
    ros::Rate loop_rate(40);

    // setup variables for publishing wheel velocities
    std_msgs::Float64 velKF1, velKF2, velKR1, velKR2;

    // define publishers
    ros::Publisher velocityKF1_pub = n.advertise<std_msgs::Float64>("/velocityKF1_send", 1000);
    ros::Publisher velocityKF2_pub = n.advertise<std_msgs::Float64>("/velocityKF2_send", 1000);
    ros::Publisher velocityKR1_pub = n.advertise<std_msgs::Float64>("/velocityKR1_send", 1000);
    ros::Publisher velocityKR2_pub = n.advertise<std_msgs::Float64>("/velocityKR2_send", 1000);

    // define subscribers
    ros::Subscriber sub1 = n.subscribe("/positionKF1_receive", 1000, &posKF1_rec);
    ros::Subscriber sub2 = n.subscribe("/positionKF2_receive", 1000, &posKF2_rec);
    ros::Subscriber sub3 = n.subscribe("/positionKR1_receive", 1000, &posKR1_rec);
    ros::Subscriber sub4 = n.subscribe("/positionKR2_receive", 1000, &posKR2_rec);
    ros::Subscriber sub5 = n.subscribe("/netft_data", 1000, &ft_rec);
    ros::Subscriber subTar = n.subscribe("/process/target", 1000, &target_pos_rec);

    //*******************************************************************************
    // Make sure to clear the robot's faults else it won't move if it's already in fault
    example_clear_faults(n, robot_name);
    //*******************************************************************************

    // move arm to start position
    VectorXd start_position(7, 1);
    start_position << 0, 0, 0, 0, 0, 0, 90; // arm straight upwards
    example_send_joint_angles(n, robot_name, degrees_of_freedom, start_position);
    std::this_thread::sleep_for(std::chrono::milliseconds(10000));

    // move arm to walker position
    VectorXd walker_position(7, 1);
    walker_position << 0, 25.3, 0, 278, 0, 328.56, 90; // Experimental values - Handles in good position
    example_send_joint_angles(n, robot_name, degrees_of_freedom, walker_position);
    std::this_thread::sleep_for(std::chrono::milliseconds(10000));

    // define a vector for joint speeds
    VectorXd kortex_speed(7, 1);

    // log start time (in seconds)
    double time_start = ros::Time::now().toSec();

    // setup file for force logging
    FILE *fpforce;
    fpforce = fopen("/home/user/Desktop/force.txt", "w");
    fprintf(fpforce, "t   fx   fy    fz  tq_x   tq_y   tq_z   f_vertical   f_front   tau\r\n");

    // declare velocity command variables
    double time_old = 0;
    float Vx_final = 0;
    float Vy_final = 0;
    float omega_final = 0;

    // for removing offset
    int window_size = 100;
    MatrixXd biased_data_history(6, window_size);
    VectorXd biased_data_points(6);
    VectorXd row_means(6);

    // Define PID controllers for object tracking
    PIDController pid_angular(Kp_angular, Ki_angular, Kd_angular, max_angular_velocity, min_angular_velocity);
    PIDController pid_distance(Kp_distance, Ki_distance, Kd_distance, max_linear_velocity, min_linear_velocity);
    auto previous_time = std::chrono::high_resolution_clock::now();

    // main loop
    while (ros::ok())
    {
        // for keeping track of time (in seconds)
        double time_now = ros::Time::now().toSec();
        double time1 = time_now - time_start;

        // Get the current time (for PID)
        auto current_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = current_time - previous_time;
        double dt = elapsed.count();

        // to remove offset from force sensor
        if (loop_count < 100)
        {
            // adds force data to matrix to be used in calibration
            biased_data_points << end_f[0], end_f[1], end_f[2], end_tq[0], end_tq[1], end_tq[2];
            biased_data_history.col(loop_count) = biased_data_points;
        }

        // calculate mean of each row
        row_means = biased_data_history.rowwise().mean();

        if (time1 > 4.0 && time1 < 6.0)
        {
            // arm setup
            kortex_speed[0] = 0 / 10.0; // TODO why divide by 10.0?
            kortex_speed[1] = 0 / 10.0;
            kortex_speed[2] = 0 / 10.0;
            kortex_speed[3] = 0 / 10.0;
            kortex_speed[4] = 0 / 10.0;
            kortex_speed[5] = 0 / 10.0;
            kortex_speed[6] = 0 / 10.0;

            example_send_joint_speeds(n, robot_name, degrees_of_freedom, kortex_speed);

            // move forward slightly, indicating robot is ready
            double Vx = 0.01;   // forward speed
            double Vy = 0.0;    // sideways speed (strafe)
            double omega = 0.0; // rotation speed
            VectorXd velocities = getWheelVelocities(Vx, Vy, omega);
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
        }
        else if (time1 > 6.0)
        {
            // regular operation begins
            kortex_speed[0] = 0;
            kortex_speed[1] = 0;
            kortex_speed[2] = 0;
            kortex_speed[3] = 0;
            kortex_speed[4] = 0;
            kortex_speed[5] = 0;
            kortex_speed[6] = 0;
            example_send_joint_speeds(n, robot_name, degrees_of_freedom, kortex_speed);

            double Vx = 0.0;    // forward speed
            double Vy = 0.0;    // sideways speed (strafe)
            double omega = 0.0; // rotation speed

            // process forces (apply offset and compensate for geometry of setup, force sensor mounted on an angle)
            double f_vertical = -0.7071 * ((end_f[0] - row_means(0)) - (end_f[1] - row_means(1))); // x-axis is in 45 and y-axis is in 135 from ground normal - 0.7071 is cos(45)
            double fz = end_f[2] - row_means(2);
            double tau_y = end_tq[1] - row_means(4);

            // ------------------------------------------------------- Robot Control Loop ----------------------------------------------------------------
            float target_x = target_pos.x;     // in px
            float target_y = target_pos.y;     // in px
            float target_depth = target_pos.z; // in mm
            // Implement control algorithm here <---------------------------------------------------------------------------------------------------------
            // Inputs available:
            // targetX - horizontal location of centroid of target object in pixel coordinates (from the perspective of the Kinect camera)
            // targetY - vertical location of centroid of target object in pixel coordinates (from the perspective of the Kinect camera)
            //    NOTE: If the target object is not found in the image, both targetX and targetY will be -1
            // targetDepth - depth of centroid of target object (in mm)
            //    NOTE: If the target object is ~500mm from the camera, the depth registers as 0.0
            //    NOTE: If the object depth cannot be found or calculated, it returns -1

            // Calculate pixel error
            double pixel_error = target_x - desired_x;

            // Convert pixel error to angular error
            double angular_error = (pixel_error / frame_width) * horizontal_fov_radians;

            // Calculate distance error
            double distance_error = target_depth - desired_distance;

            // Calculate error range in radians
            double accepted_error_range_radians = (accepted_error_range_pixels / frame_width) * horizontal_fov_radians;

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

            // Check if distance error is within the accepted range
            if (std::abs(distance_error) > accepted_distance_error)
            {
                // Compute control output using PID controller for linear velocity
                Vx = pid_distance.compute(distance_error, dt);
            }
            else
            {
                std::string ros_msg = "Distance within accepted range. No distance correction needed.";
                ROS_INFO("%s", ros_msg.c_str());
            }

            // calculate individual wheel speeds
            VectorXd velocities = getWheelVelocities(Vx_final, Vy_final, omega_final); // calculate individual wheel velocities
            VectorXd wheel_speeds = velocities * 109 / pi;
            double max_speed = wheel_speeds.cwiseAbs().maxCoeff();

            // Scale wheel speeds if the maximum exceeds the allowed limit
            if (max_speed > max_wheel_speed)
            {
                wheel_speeds = wheel_speeds * (max_wheel_speed / max_speed);
            }
            // Update the previous time
            previous_time = current_time;
            
            // TODO make this function run independantly of the arm holding position and only when the speed is updated

            /*
            if (fz < -f_thresh) // fz < -5
            {
                // move forward
                Vx = 0.30;
                Vy = 0.0;
                omega = 0.0;

                // turn robot if torque is sufficient
                // this will result in the robot driving through a curve, as the robot is also moving forward
                if (tau_y > t_thresh) // tau_y > 2.5
                {
                    omega = -0.3;
                }
                else if (tau_y < -t_thresh) // tau_y < -2.5
                {
                    omega = 0.3;
                }
            }
            else if (fz > f_thresh) // fz > 5
            {
                // move backwards
                Vx = -0.30;
                Vy = 0.0;
                omega = 0.0;
            }
            else
            {
                // if force is below threshold, and torque is above threshold, turn robot in place
                if (tau_y > t_thresh) // tau_y > 2.5
                {
                    Vx = 0.0;
                    // Vy = 0.0;
                    omega = -0.5;
                }
                else if (tau_y < -t_thresh) // tau_y < -2.5
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

            */

            // -------------------------------------------------------------------------------------------------------------------------------------------

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
        }

        time_old = time_now;

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}