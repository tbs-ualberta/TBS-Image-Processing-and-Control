// Andor Siegers and Mahdi Chalaki - based off an example provided by Kinova for their gen3 arm
#include "wmm_control/HelperFunctions.h"
#include "ros/ros.h"
#include <kortex_driver/Base_ClearFaults.h>
#include <kortex_driver/PlayJointTrajectory.h>
#include <kortex_driver/CartesianSpeed.h>
#include <kortex_driver/BaseCyclic_Feedback.h>
#include <kortex_driver/ReadAction.h>
#include <kortex_driver/ExecuteAction.h>
#include <kortex_driver/PlayJointTrajectory.h>
#include <kortex_driver/SendJointSpeedsCommand.h>
#include <kortex_driver/SetCartesianReferenceFrame.h>
#include <kortex_driver/CartesianReferenceFrame.h>
#include <kortex_driver/SendGripperCommand.h>
#include <kortex_driver/GripperMode.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Dense>
#include <csignal>
#include <iostream>
#include <thread>
#include <chrono>

using namespace Eigen;
using namespace std;

#define HOME_ACTION_IDENTIFIER 2
#define pi 3.1416

// Global constant definitions
// For mobile base wheels velocity calculation
const double R = constants::RADIUS;     // Radius of the wheels
const double W = constants::WIDTH;      // Width of robot's wheelbase
const double L = constants::LENGTH;     // Length of robot's wheelbase
const double LAMBDA = constants::LAMBDA;

double PIDController::compute(double error, double dt)
{
    integral += error * dt;
    double derivative = (error - previous_error) / dt;
    double output = Kp * error + Ki * integral + Kd * derivative;

    if (output > max_output)
        output = max_output;
    else if (output < min_output)
        output = min_output;

    previous_error = error;
    return output;
} // end compute

void clear_faults(ros::NodeHandle n, std::string robot_name)
{
    ros::ServiceClient service_client_clear_faults = n.serviceClient<kortex_driver::Base_ClearFaults>("/" + robot_name + "/base/clear_faults");
    kortex_driver::Base_ClearFaults service_clear_faults;

    // Clear the faults
    if (!service_client_clear_faults.call(service_clear_faults))
    {
        std::string error_string = "Failed to clear the faults";
        ROS_ERROR("%s", error_string.c_str());
        throw new std::runtime_error(error_string);
    }

    // Wait a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
} // end clear_faults

void send_joint_angles(ros::NodeHandle n, std::string robot_name, int degrees_of_freedom, VectorXd position_gen3)
{
    // Initialize the ServiceClient
    ros::ServiceClient service_client_play_joint_trajectory = n.serviceClient<kortex_driver::PlayJointTrajectory>("/" + robot_name + "/base/play_joint_trajectory");
    kortex_driver::PlayJointTrajectory service_play_joint_trajectory;

    VectorXd angles_to_send(7, 1);
    for (unsigned int i = 0; i < degrees_of_freedom; i++)
    {
        angles_to_send[i] = position_gen3(i);
    }

    for (int i = 0; i < degrees_of_freedom; i++)
    {
        kortex_driver::JointAngle temp_angle;
        temp_angle.joint_identifier = i;
        temp_angle.value = angles_to_send[i];
        service_play_joint_trajectory.request.input.joint_angles.joint_angles.push_back(temp_angle);
    }

    if (service_client_play_joint_trajectory.call(service_play_joint_trajectory))
    {
        ROS_INFO("The joint angles were sent to the robot.");
    }
    else
    {
        std::string error_string = "Failed to call PlayJointTrajectory";
        ROS_ERROR("%s", error_string.c_str());
        throw new std::runtime_error(error_string);
    }
} // end send_joint_angles

void send_joint_speeds(ros::NodeHandle n, std::string robot_name, int degrees_of_freedom, VectorXd speed_gen3)
{
    // Initialize the ServiceClient
    ros::ServiceClient service_client_send_joint_speed = n.serviceClient<kortex_driver::SendJointSpeedsCommand>("/" + robot_name + "/base/send_joint_speeds_command");
    kortex_driver::SendJointSpeedsCommand service_send_joint_speed;

    VectorXd speeds_to_send(7, 1);
    for (unsigned int i = 0; i < degrees_of_freedom; i++)
    {
        speeds_to_send[i] = speed_gen3(i);
    }

    for (int i = 0; i < degrees_of_freedom; i++)
    {
        kortex_driver::JointSpeed temp_speed;
        temp_speed.joint_identifier = i;
        temp_speed.value = speeds_to_send[i];
        service_send_joint_speed.request.input.joint_speeds.push_back(temp_speed);
    }

    if (service_client_send_joint_speed.call(service_send_joint_speed))
    {
    }
    else
    {
        std::string error_string = "Failed to call PlayJointTrajectory";
        ROS_ERROR("%s", error_string.c_str());
        throw new std::runtime_error(error_string);
    }
} // end send_joint_speeds

void get_arm_info(std::string* robot_name, int* degrees_of_freedom, bool* is_gripper_present)
{
    // Ensure parameters are correct and print them to the info stream
    // Parameter robot_name
    if (!ros::param::get("/my_gen3/my_gen3_driver/robot_name", *robot_name))
    {
        std::string error_string = "Parameter robot_name was not specified, defaulting to " + *robot_name + " as namespace";
        ROS_WARN("%s", error_string.c_str());
    }
    else
    {
        std::string error_string = "Using robot_name " + *robot_name + " as namespace";
        ROS_INFO("%s", error_string.c_str());
    }

    // Parameter degrees_of_freedom
    if (!ros::param::get("/" + *robot_name + "/degrees_of_freedom", *degrees_of_freedom))
    {
        std::string error_string = "Parameter /" + *robot_name + "/degrees_of_freedom was not specified, defaulting to " + std::to_string(*degrees_of_freedom) + " as degrees of freedom";
        ROS_WARN("%s", error_string.c_str());
    }
    else
    {
        std::string error_string = "Using degrees_of_freedom " + std::to_string(*degrees_of_freedom) + " as degrees_of_freedom";
        ROS_INFO("%s", error_string.c_str());
    }

    // Parameter is_gripper_present
    if (!ros::param::get("/" + *robot_name + "/is_gripper_present", *is_gripper_present))
    {
        std::string error_string = "Parameter /" + *robot_name + "/is_gripper_present was not specified, defaulting to " + std::to_string(*is_gripper_present);
        ROS_WARN("%s", error_string.c_str());
    }
    else
    {
        std::string error_string = "Using is_gripper_present " + std::to_string(*is_gripper_present);
        ROS_INFO("%s", error_string.c_str());
    }
} // end get_arm_info

// These functions update global variables with incoming data from the subscribed topics
void posKF1_rec(const std_msgs::Float64 &posKF1_msg)
{
    pos_base[0] = posKF1_msg.data;
}
void posKF2_rec(const std_msgs::Float64 &posKF2_msg)
{
    pos_base[1] = posKF2_msg.data;
}
void posKR1_rec(const std_msgs::Float64 &posKR1_msg)
{
    pos_base[2] = posKR1_msg.data;
}
void posKR2_rec(const std_msgs::Float64 &posKR2_msg)
{
    pos_base[3] = posKR2_msg.data;
}
void target_pos_rec(const geometry_msgs::Point &tar_pos_msg)
{
    target_pos = tar_pos_msg;
    update_target = true;
}
void ft_rec(const geometry_msgs::WrenchStamped &ft_msg)
{
    end_f[0] = ft_msg.wrench.force.x;
    end_f[1] = ft_msg.wrench.force.y;
    end_f[2] = ft_msg.wrench.force.z;

    end_tq[0] = ft_msg.wrench.torque.x;
    end_tq[1] = ft_msg.wrench.torque.y;
    end_tq[2] = ft_msg.wrench.torque.z;
}

// Stopping the wheels after terminating the program with ctrl+C
void signalHandler(int signum)
{
    if (signum == SIGINT)
    {
        std_msgs::Float64 velKF1, velKF2, velKR1, velKR2;
        ros::NodeHandle n("~");
        std::string robot_name = "my_gen3";

        ros::Publisher velocityKF1_pub = n.advertise<std_msgs::Float64>("/velocityKF1_send", 1000);
        ros::Publisher velocityKF2_pub = n.advertise<std_msgs::Float64>("/velocityKF2_send", 1000);
        ros::Publisher velocityKR1_pub = n.advertise<std_msgs::Float64>("/velocityKR1_send", 1000);
        ros::Publisher velocityKR2_pub = n.advertise<std_msgs::Float64>("/velocityKR2_send", 1000);

        velKF1.data = 0;
        velKF2.data = 0;
        velKR1.data = 0;
        velKR2.data = 0;
        velocityKF1_pub.publish(velKF1);
        velocityKF2_pub.publish(velKF2);
        velocityKR1_pub.publish(velKR1);
        velocityKR2_pub.publish(velKR2);

        VectorXd start_position(7, 1);
        start_position << 0, 0, 0, 0, 0, 0, 90;
        send_joint_angles(n, robot_name, 7, start_position);

        // Terminate the program
        exit(signum);
    }
} // end signalHandler

// Calculates the corresponding velocities of each wheel to produce the given base velocity
Eigen::VectorXd getWheelVelocities(double V_x, double V_y, double omega)
{
    VectorXd wheel_speeds(4, 1);
    wheel_speeds << 0, 0, 0, 0; // Initialize with 4 zeros
    wheel_speeds[0] = (1.0 / R) * (V_x - V_y - omega * LAMBDA); // V_fl
    wheel_speeds[1] = (1.0 / R) * (V_x + V_y + omega * LAMBDA); // V_fr
    wheel_speeds[2] = (1.0 / R) * (V_x + V_y - omega * LAMBDA); // V_rl
    wheel_speeds[3] = (1.0 / R) * (V_x - V_y + omega * LAMBDA); // V_rr

    return wheel_speeds;
} // end getWheelVelocities

// Calculates robot velocity from wheel speeds
geometry_msgs::Twist getRobotVelocities(double V_fl, double V_fr, double V_rl, double V_rr)
{
    geometry_msgs::Twist twist;
    double V_x, V_y, omega;

    // Calculate the velocities
    V_x = (R / 4) * (V_fl + V_fr + V_rl + V_rr);
    V_y = (R / 4) * (-V_fl + V_fr + V_rl - V_rr);
    omega = (R / (4 * LAMBDA)) * (-V_fl + V_fr - V_rl + V_rr);

    // Set the calculated velocities to the twist message
    twist.linear.x = V_x;
    twist.linear.y = V_y;
    twist.linear.z = 0.0;  // Assuming no movement in the z direction
    twist.angular.x = 0.0; // Assuming no rotation around the x axis
    twist.angular.y = 0.0; // Assuming no rotation around the y axis
    twist.angular.z = omega;

    return twist;
} // end getRobotVelocities