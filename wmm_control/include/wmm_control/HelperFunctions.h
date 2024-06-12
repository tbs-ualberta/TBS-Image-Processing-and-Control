#ifndef HELPER_FUNCTIONS_H
#define HELPER_FUNCTIONS_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/WrenchStamped.h>

// global constant definitions
namespace constants
{
    // for mobile base wheels velocity calculation
    const double RADIUS = 0.05; // Radius of the wheels
    const double WIDTH = 0.21; // Width of robot's wheelbase
    const double LENGTH = 0.21; // Length of robot's wheelbase
    const double LAMBDA = (WIDTH + LENGTH) / 2.0;
}

// global variable definitions
extern double end_f[3], end_tq[3];
extern double pos_base[4];
extern geometry_msgs::Point target_pos; // Defines the position of the target in x(px), y(px), z(mm)
extern bool update_target;

class PIDController
{
public:
    PIDController(double Kp, double Ki, double Kd, double max_output, double min_output)
        : Kp(Kp), Ki(Ki), Kd(Kd), max_output(max_output), min_output(min_output), previous_error(0), integral(0) {}

    double compute(double error, double dt);

private:
    double Kp;
    double Ki;
    double Kd;
    double max_output;
    double min_output;
    double previous_error;
    double integral;
};

void example_clear_faults(ros::NodeHandle n, std::string robot_name);
void example_send_joint_angles(ros::NodeHandle n, std::string robot_name, int degrees_of_freedom, Eigen::VectorXd position_gen3);
void example_send_joint_speeds(ros::NodeHandle n, std::string robot_name, int degrees_of_freedom, Eigen::VectorXd speed_gen3);
void get_arm_info(std::string* robot_name, int* degrees_of_freedom, bool* is_gripper_present);
void posKF1_rec(const std_msgs::Float64 &posKF1_msg);
void posKF2_rec(const std_msgs::Float64 &posKF2_msg);
void posKR1_rec(const std_msgs::Float64 &posKR1_msg);
void posKR2_rec(const std_msgs::Float64 &posKR2_msg);
void target_pos_rec(const geometry_msgs::Point &tar_pos_msg);
void ft_rec(const geometry_msgs::WrenchStamped &ft_msg);
void signalHandler(int signum);
Eigen::VectorXd getWheelVelocities(double V_x, double V_y, double omega);
geometry_msgs::Twist getRobotVelocities(double V_fl, double V_fr, double V_rl, double V_rr);

#endif // HELPER_FUNCTIONS_H