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

// Camera parameters - from: https://www.researchgate.net/publication/283482095_Calibration_of_Kinect_for_Xbox_One_and_Comparison_between_the_Two_Generations_of_Microsoft_Sensors
const int FRAME_WITDH = 1920;
const int FRAME_HEIGHT = 1080;
const double HORIZONTAL_FOV_DEG = 70;
const double VERTICAL_FOV_DEG = 60;

// Arm parameters
const double DEFAULT_POINTER_ANG_HORIZONTAL = 0;
const double DEFAULT_POINTER_ANG_VERTICAL = 135;

// Global variable definitions
double end_f[3], end_tq[3];
double pos_base[4];
geometry_msgs::Point target_pos; // Holds the position of the target in x(px), y(px), z(mm)
bool update_target = false;

int loop_count = 0;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "wmm_pointer_node");
    ros::NodeHandle n("~");

    // Kinova arm Parameters
    string robot_name = "my_gen3";
    int degrees_of_freedom = 7;
    bool is_gripper_present = false;

    // Get Kinova arm information and set to defaults
    get_arm_info(&robot_name, &degrees_of_freedom, &is_gripper_present);

    signal(SIGINT, signalHandler);

    // Initialize default arm position
    VectorXd default_arm_angles(7, 1);
    // default_arm_angles << DEFAULT_POINTER_ANG_HORIZONTAL, -45, 0, DEFAULT_POINTER_ANG_VERTICAL, 0, 0, -90; // pointing position
    default_arm_angles << DEFAULT_POINTER_ANG_HORIZONTAL, 315, 0, DEFAULT_POINTER_ANG_VERTICAL, 0, 0, 270; // pointing position

    // Set loop rate
    ros::Rate loop_rate(40);

    // Define subscriber to find target
    ros::Subscriber subTar = n.subscribe("/process/target", 1000, &target_pos_rec);

    // Make sure to clear the robot's faults else it won't move if it's already in fault
    clear_faults(n, robot_name);

    // Move arm to default position
    send_joint_angles(n, robot_name, degrees_of_freedom, default_arm_angles);
    // std::this_thread::sleep_for(std::chrono::milliseconds(10000));
    std::this_thread::sleep_for(std::chrono::milliseconds(3000)); // for testing

    // Define a vector for joint speeds
    VectorXd kortex_speed(7, 1);

    // Main loop
    while (ros::ok())
    {
        if(update_target)
        {
            // Separate target position into 3 variables
            float target_x = target_pos.x;     // in px
            float target_y = target_pos.y;     // in px
            float target_depth = target_pos.z; // in mm
            target_depth = target_depth / 1000;// convert to meters

            std::cout << "Target position: (" << target_x << ", " << target_y << ", " << target_depth << ")" << std::endl;
            
            // Initialize new joint angles in default position
            VectorXd new_joint_angles(7);
            new_joint_angles = default_arm_angles;

            // Check if valid target
            if(target_x == -1 && target_y == -1)
            {
                // Target not in frame
                std::cout << "Target not found." << std::endl;
            }
            else
            {
                // If target in frame, turn towards target

                // Calculate pixel error
                double pixels_from_center_x = target_x - FRAME_WITDH / 2;
                double pixels_from_center_y = target_y - FRAME_HEIGHT / 2;

                // Convert pixel error to angular error
                double angular_dif_x = (pixels_from_center_x / FRAME_WITDH) * HORIZONTAL_FOV_DEG;
                double angular_dif_y = (pixels_from_center_y / FRAME_HEIGHT) * VERTICAL_FOV_DEG;

                // Calculate new angles for joints
                double new_pointer_angle_x = DEFAULT_POINTER_ANG_HORIZONTAL - angular_dif_x; // subtraction because camera is flipped horizontally
                double new_pointer_angle_y = DEFAULT_POINTER_ANG_VERTICAL + angular_dif_y;   // addition because camera is not flipped vertically

                if(new_pointer_angle_x < 0) new_pointer_angle_x += 360;
                if(new_pointer_angle_y < 0) new_pointer_angle_y += 360;

                // Update vectors
                new_joint_angles(0) = new_pointer_angle_x;
                new_joint_angles(3) = new_pointer_angle_y;
            }

            // Send angles to the robot
            send_joint_angles(n, robot_name, degrees_of_freedom, new_joint_angles);

            // Set update to false, so that the loop doesn't run again until the target position is updated
            update_target = false;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
} // end main

// TODO make it point vertically as well