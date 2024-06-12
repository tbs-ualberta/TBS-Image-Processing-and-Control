#include "wmm_control/HelperFunctions.h"
#include "ros/ros.h"
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
const double R = constants::RADIUS; // Radius of the wheels
const double W = constants::WIDTH; // Width of robot's wheelbase
const double L = constants::LENGTH; // Length of robot's wheelbase
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
}

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

        // Terminate the program
        exit(signum);
    }
}

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
}

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
}