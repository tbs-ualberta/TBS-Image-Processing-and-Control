// Andor Siegers and Mahdi Chalaki - based off an example provided by Kinova for their gen3 arm

// Purely for testing the functionality of the other programs in this package in coordination with gazebo
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

// Global variable definitions (useless for this program but needed to compile)
double end_f[3], end_tq[3];
double pos_base[4];
geometry_msgs::Point target_pos; // Holds the position of the target in x(px), y(px), z(mm)
bool update_target = false;

int loop_count = 0;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "wave_arm_node");
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

    // Make sure to clear the robot's faults else it won't move if it's already in fault
    clear_faults(n, robot_name);

    // Main loop
    while (ros::ok())
    {
        // For keeping track of time (in seconds)
        // Move arm to start position
        VectorXd start_position(7, 1);
        start_position << 0, 0, 0, 0, 0, 0, 90; // arm straight upwards
        send_joint_angles(n, robot_name, degrees_of_freedom, start_position);
        std::this_thread::sleep_for(std::chrono::milliseconds(5000)); // for testing

        // Move arm to walker position
        VectorXd walker_position(7, 1);
        walker_position << 0, 25.3, 0, 278, 0, 328.56, 90; // Experimental values - Handles in good position
        send_joint_angles(n, robot_name, degrees_of_freedom, walker_position);
        std::this_thread::sleep_for(std::chrono::milliseconds(5000)); // for testing

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
} // end main

// TODO write launch file to launch WMM
// Write documentation that explains launch file