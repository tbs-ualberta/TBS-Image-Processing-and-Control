/*
 * KINOVA (R) KORTEX (TM)
 *
 * Copyright (c) 2019 Kinova inc. All rights reserved.
 *
 * This software may be modified and distributed
 * under the terms of the BSD 3-Clause license.
 *
 * Refer to the LICENSE file for details.
 *
 */
#include <thread>

#include "ros/ros.h"
#include <kortex_driver/Base_ClearFaults.h>
#include <kortex_driver/PlayCartesianTrajectory.h>
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

#include <iostream>
#include <Eigen/Dense>
#include <ros/time.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"

#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/TwistStamped.h"

#include <csignal>
#include <unistd.h>

using namespace Eigen;
using namespace std;

#define HOME_ACTION_IDENTIFIER 2
#define pi 3.1416

double end_f[3], end_tq[3];
double pos[7],vel[7],tor[7],pos_base[4];

void example_clear_faults(ros::NodeHandle n, std::string robot_name)
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
}

void example_home_the_robot(ros::NodeHandle n, std::string robot_name) // not used currently
{
  ros::ServiceClient service_client_read_action = n.serviceClient<kortex_driver::ReadAction>("/" + robot_name + "/base/read_action");
  kortex_driver::ReadAction service_read_action;

  // The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
  service_read_action.request.input.identifier = HOME_ACTION_IDENTIFIER;

  if (!service_client_read_action.call(service_read_action))
  {
    std::string error_string = "Failed to call ReadAction";
    ROS_ERROR("%s", error_string.c_str());
    throw new std::runtime_error(error_string);
  }

  // We can now execute the Action that we read 
  ros::ServiceClient service_client_execute_action = n.serviceClient<kortex_driver::ExecuteAction>("/" + robot_name + "/base/execute_action");
  kortex_driver::ExecuteAction service_execute_action;

  service_execute_action.request.input = service_read_action.response.output;
  
  if (service_client_execute_action.call(service_execute_action))
  {
    ROS_INFO("The Home position action was sent to the robot.");
  }
  else
  {
    std::string error_string = "Failed to call ExecuteAction";
    ROS_ERROR("%s", error_string.c_str());
    throw new std::runtime_error(error_string);
  }
}

void example_set_cartesian_reference_frame(ros::NodeHandle n, std::string robot_name) // not used currently
{
  // Initialize the ServiceClient
  ros::ServiceClient service_client_set_cartesian_reference_frame = n.serviceClient<kortex_driver::SetCartesianReferenceFrame>("/" + robot_name + "/control_config/set_cartesian_reference_frame");
  kortex_driver::SetCartesianReferenceFrame service_set_cartesian_reference_frame;

  service_set_cartesian_reference_frame.request.input.reference_frame = kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_MIXED;
  if (!service_client_set_cartesian_reference_frame.call(service_set_cartesian_reference_frame))
  {
    std::string error_string = "Failed to call SetCartesianReferenceFrame";
    ROS_ERROR("%s", error_string.c_str());
    throw new std::runtime_error(error_string);
  }

  // Wait a bit
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
}

// Moves the robot to the specified cartesion coordinate
void example_send_cartesian_pose(ros::NodeHandle n, std::string robot_name) // not used currently
{
  // Get the actual cartesian pose to increment it
  // You can create a subscriber to listen to the base_feedback
  // Here we only need the latest message in the topic though
  auto feedback = ros::topic::waitForMessage<kortex_driver::BaseCyclic_Feedback>("/" + robot_name + "/base_feedback");

  // Initialize the ServiceClient for the cartesian pose
  ros::ServiceClient service_client_play_cartesian_trajectory = n.serviceClient<kortex_driver::PlayCartesianTrajectory>("/" + robot_name + "/base/play_cartesian_trajectory");
  kortex_driver::PlayCartesianTrajectory service_play_cartesian_trajectory;

  // Initialize input
  float current_x = feedback->base.commanded_tool_pose_x;
  float current_y = feedback->base.commanded_tool_pose_y;
  float current_z = feedback->base.commanded_tool_pose_z;
  float current_theta_x = feedback->base.commanded_tool_pose_theta_x;
  float current_theta_y = feedback->base.commanded_tool_pose_theta_y;
  float current_theta_z = feedback->base.commanded_tool_pose_theta_z;

  // Creating the target pose
  service_play_cartesian_trajectory.request.input.target_pose.x = current_x;
  service_play_cartesian_trajectory.request.input.target_pose.y = current_y;
  service_play_cartesian_trajectory.request.input.target_pose.z = current_z + 0.15;
  service_play_cartesian_trajectory.request.input.target_pose.theta_x = current_theta_x;
  service_play_cartesian_trajectory.request.input.target_pose.theta_y = current_theta_y;
  service_play_cartesian_trajectory.request.input.target_pose.theta_z = current_theta_z + 35;

  kortex_driver::CartesianSpeed poseSpeed;
  poseSpeed.translation = 0.1;
  poseSpeed.orientation = 15;

  // The constraint is a one_of in Protobuf. The one_of concept does not exist in ROS
  // To specify a one_of, create it and put it in the appropriate vector of the oneof_type member of the ROS object : 
  service_play_cartesian_trajectory.request.input.constraint.oneof_type.speed.push_back(poseSpeed);

  if (service_client_play_cartesian_trajectory.call(service_play_cartesian_trajectory))
  {
    ROS_INFO("The new cartesian pose was sent to the robot.");
  }
  else
  {
    std::string error_string = "Failed to call PlayCartesianTrajectory";
    ROS_ERROR("%s", error_string.c_str());
    throw new std::runtime_error(error_string);
  }
}

void example_send_joint_angles(ros::NodeHandle n, std::string robot_name, int degrees_of_freedom, VectorXd position_gen3)
{
  // Initialize the ServiceClient
  ros::ServiceClient service_client_play_joint_trajectory = n.serviceClient<kortex_driver::PlayJointTrajectory>("/" + robot_name + "/base/play_joint_trajectory");
  kortex_driver::PlayJointTrajectory service_play_joint_trajectory;

  //std::vector<double> angles_to_send;
  /*for (unsigned int i = 0; i < degrees_of_freedom-1; i++)
  {
    angles_to_send.push_back(0.0);
  }
  angles_to_send[6]=30;*/

  VectorXd angles_to_send(7,1);
  for (unsigned int i = 0; i < degrees_of_freedom; i++)
  {
    angles_to_send[i]=position_gen3(i);
  }
  //angles_to_send[6]=30;
  //cout << position_gen3(1) << endl;
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
}

void example_send_joint_speeds(ros::NodeHandle n, std::string robot_name, int degrees_of_freedom, VectorXd speed_gen3)
{
  // Initialize the ServiceClient
  ros::ServiceClient service_client_send_joint_speed = n.serviceClient<kortex_driver::SendJointSpeedsCommand>("/" + robot_name + "/base/send_joint_speeds_command");
  kortex_driver::SendJointSpeedsCommand service_send_joint_speed;


  VectorXd speeds_to_send(7,1);
  for (unsigned int i = 0; i < degrees_of_freedom; i++)
  {
    speeds_to_send[i]=speed_gen3(i);
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

}

void example_send_gripper_command(ros::NodeHandle n, std::string robot_name, double value) // not in use currently
{
  // Initialize the ServiceClient
  ros::ServiceClient service_client_send_gripper_command = n.serviceClient<kortex_driver::SendGripperCommand>("/" + robot_name + "/base/send_gripper_command");
  kortex_driver::SendGripperCommand service_send_gripper_command;

  // Initialize the request
  kortex_driver::Finger finger;
  finger.finger_identifier = 0;
  finger.value = value;
  service_send_gripper_command.request.input.gripper.finger.push_back(finger);
  service_send_gripper_command.request.input.mode = kortex_driver::GripperMode::GRIPPER_POSITION;

  if (service_client_send_gripper_command.call(service_send_gripper_command))  
  {
    ROS_INFO("The gripper command was sent to the robot.");
  }
  else
  {
    std::string error_string = "Failed to call SendGripperCommand";
    ROS_ERROR("%s", error_string.c_str());
    throw new std::runtime_error(error_string);
  }
}

// These functions process incoming data from the subscribed topics
void posKF1_rec(const std_msgs::Float64& posKF1_msg)
{
  pos_base[0]=posKF1_msg.data;
}
void posKF2_rec(const std_msgs::Float64& posKF2_msg)
{
  pos_base[1]=posKF2_msg.data;
}
void posKR1_rec(const std_msgs::Float64& posKR1_msg)
{
  pos_base[2]=posKR1_msg.data;
}
void posKR2_rec(const std_msgs::Float64& posKR2_msg)
{
  pos_base[3]=posKR2_msg.data;
}

void ft_rec(const geometry_msgs::WrenchStamped &ft_msg)
{
  end_f[0]=ft_msg.wrench.force.x;
  end_f[1]=ft_msg.wrench.force.y;
  end_f[2]=ft_msg.wrench.force.z;

  end_tq[0]=ft_msg.wrench.torque.x;
  end_tq[1]=ft_msg.wrench.torque.y;
  end_tq[2]=ft_msg.wrench.torque.z;
}

void JointstatesCallback(const sensor_msgs::JointStateConstPtr& msg) // not used currently
{
 // pos=msg.position;
 // float ;
  pos[0]=msg->position[0];
  pos[1]=msg->position[1];
  pos[2]=msg->position[2];
  pos[3]=msg->position[3];
  pos[4]=msg->position[4];
  pos[5]=msg->position[5];
  pos[6]=msg->position[6];
  vel[0]=msg->velocity[0];
  vel[1]=msg->velocity[1];
  vel[2]=msg->velocity[2];
  vel[3]=msg->velocity[3];
  vel[4]=msg->velocity[4];
  vel[5]=msg->velocity[5];
  vel[6]=msg->velocity[6];
  tor[0]=msg->effort[0];
  tor[1]=msg->effort[1];
  tor[2]=msg->effort[2];
  tor[3]=msg->effort[3];
  tor[4]=msg->effort[4];
  tor[5]=msg->effort[5];
  tor[6]=msg->effort[6];
  //cout << msg->position[0] <<endl;
}

// Mobile base wheels velocity calculation
const double r = 0.05; // Radius of the wheels
const double W = 0.21; // Width of robot's wheelbase
const double L = 0.21; // Length of the robot
const double lambda = (W + L) / 2.0;

// calculates the corresponding velocities of each wheel to produce the given base velocity
VectorXd getWheelVelocities(double V_x, double V_y, double omega)
{
  VectorXd wheelSpeeds(4, 1);
  wheelSpeeds << 0,0,0,0; // Initialize with 4 zeros

  wheelSpeeds[0] = (1.0/r) * (V_x - V_y - omega * lambda); // V_FL
  wheelSpeeds[1] = (1.0/r) * (V_x + V_y + omega * lambda); // V_FR
  wheelSpeeds[2] = (1.0/r) * (V_x + V_y - omega * lambda); // V_RL
  wheelSpeeds[3] = (1.0/r) * (V_x - V_y + omega * lambda); // V_RR

  return wheelSpeeds;
}

// Stopping the wheels after terminating the program with ctrl+C
void signalHandler(int signum)
{
  if (signum == SIGINT)
  {
    std_msgs::Float64 velKF1,velKF2,velKR1,velKR2;
    ros::NodeHandle n("~");
    std::string robot_name = "my_gen3";

    ros::Publisher velocityKF1_pub = n.advertise<std_msgs::Float64>("/velocityKF1_send", 1000);
    ros::Publisher velocityKF2_pub = n.advertise<std_msgs::Float64>("/velocityKF2_send", 1000);
    ros::Publisher velocityKR1_pub = n.advertise<std_msgs::Float64>("/velocityKR1_send", 1000);
    ros::Publisher velocityKR2_pub = n.advertise<std_msgs::Float64>("/velocityKR2_send", 1000);

    velKF1.data=0;
    velKF2.data=0;
    velKR1.data=0;
    velKR2.data=0;
    velocityKF1_pub.publish(velKF1);
    velocityKF2_pub.publish(velKF2);
    velocityKR1_pub.publish(velKR1);
    velocityKR2_pub.publish(velKR2);

    VectorXd start_position(7,1);
    start_position << 0,0,0,0,0,0,90;
    example_send_joint_angles(n, robot_name, 7,start_position);

    // Terminate the program
    exit(signum);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Test_base");

  // ROS Parameters
  ros::NodeHandle n("~");
  std::string robot_name = "my_gen3";
  int degrees_of_freedom = 7;
  bool is_gripper_present = false;

  signal(SIGINT, signalHandler);

  // set loop rate
  ros::Rate loop_rate(40);

  // setup variables for publishing wheel velocities
  std_msgs::Float64 velKF1,velKF2,velKR1,velKR2;

  // define publishers
  ros::Publisher velocityKF1_pub = n.advertise<std_msgs::Float64>("/velocityKF1_send", 1000);
  ros::Publisher velocityKF2_pub = n.advertise<std_msgs::Float64>("/velocityKF2_send", 1000);
  ros::Publisher velocityKR1_pub = n.advertise<std_msgs::Float64>("/velocityKR1_send", 1000);
  ros::Publisher velocityKR2_pub = n.advertise<std_msgs::Float64>("/velocityKR2_send", 1000);
  // define subscribers
  ros::Subscriber sub1 = n.subscribe("/positionKF1_receive",1000,&posKF1_rec);
  ros::Subscriber sub2 = n.subscribe("/positionKF2_receive",1000,&posKF2_rec);
  ros::Subscriber sub3 = n.subscribe("/positionKR1_receive",1000,&posKR1_rec);
  ros::Subscriber sub4 = n.subscribe("/positionKR2_receive",1000,&posKR2_rec);
  
  //   ros::Subscriber sub5 = n.subscribe("/data",1000,&data_rec);  //subscribe information from the camera
  //   ros::Subscriber sub = n.subscribe("/my_gen3/joint_states",1000,&JointstatesCallback);
  

  // Ensure parameters are correct and print them to the info stream
  // Parameter robot_name
  if (!ros::param::get("~robot_name", robot_name))
  {
    std::string error_string = "Parameter robot_name was not specified, defaulting to " + robot_name + " as namespace";
    ROS_WARN("%s", error_string.c_str());
  }
  else 
  {
    std::string error_string = "Using robot_name " + robot_name + " as namespace";
    ROS_INFO("%s", error_string.c_str());
  }

  // Parameter degrees_of_freedom
  if (!ros::param::get("/" + robot_name + "/degrees_of_freedom", degrees_of_freedom))
  {
    std::string error_string = "Parameter /" + robot_name + "/degrees_of_freedom was not specified, defaulting to " + std::to_string(degrees_of_freedom) + " as degrees of freedom";
    ROS_WARN("%s", error_string.c_str());
  }
  else 
  {
    std::string error_string = "Using degrees_of_freedom " + std::to_string(degrees_of_freedom) + " as degrees_of_freedom";
    ROS_INFO("%s", error_string.c_str());
  }

  // Parameter is_gripper_present
  if (!ros::param::get("/" + robot_name + "/is_gripper_present", is_gripper_present))
  {
    std::string error_string = "Parameter /" + robot_name + "/is_gripper_present was not specified, defaulting to " + std::to_string(is_gripper_present);
    ROS_WARN("%s", error_string.c_str());
  }
  else 
  {
    std::string error_string = "Using is_gripper_present " + std::to_string(is_gripper_present);
    ROS_INFO("%s", error_string.c_str());
  }
  //*******************************************************************************
  // Make sure to clear the robot's faults else it won't move if it's already in fault
  example_clear_faults(n, robot_name);
  //*******************************************************************************
  // Move the robot to the Home position with an Action
  //example_home_the_robot(n, robot_name);
  //std::this_thread::sleep_for(std::chrono::milliseconds(10000));
  //*******************************************************************************

  // move robot to start position
  VectorXd start_position(7,1);
  start_position << 0,0,0,0,0,0,90; // arm straight upwards
  example_send_joint_angles(n, robot_name, degrees_of_freedom,start_position);
  std::this_thread::sleep_for(std::chrono::milliseconds(10000));

  // move robot to walker position
  VectorXd walker_position(7,1);
  walker_position << 0,25.3,0,278,0,328.56,90; // Experimental values - Handles in good position 
  example_send_joint_angles(n, robot_name, degrees_of_freedom, walker_position);
  std::this_thread::sleep_for(std::chrono::milliseconds(10000));

  // define a vector for joint speeds
  VectorXd kortex_speed(7,1);

  // log start time (in seconds)
  double time_start = ros::Time::now().toSec();

  // setup file for force logging
  FILE *fpforce;
  fpforce = fopen("/home/user/Desktop/force.txt","w");
  fprintf(fpforce, "t   fx   fy    fz  tq_x   tq_y   tq_z   f_vertical   f_front   tau\r\n");

  // declare velocity command variables
  double time_old = 0;
  float Vx_final = 0;
  float Vy_final = 0;
  float omega_final = 0;

  // for removing offset
  int window_size = 100;
  MatrixXd biased_data_history(6,window_size);
  VectorXd biased_data_points(6);
  VectorXd row_means(6);

	int count = 0;

  // main loop
  while(ros::ok())
  {
    // for keeping track of time (in seconds)
    double time_now = ros::Time::now().toSec();
    double time1 = time_now - time_start;

    // to remove offset from force sensor
    if (count<100)
    {
      // adds force data to matrix to be used in calibration
      biased_data_points << end_f[0],end_f[1],end_f[2],end_tq[0],end_tq[1],end_tq[2];
      biased_data_history.col(count) = biased_data_points;
    }

    // calculate mean of each row
    row_means = biased_data_history.rowwise().mean();

    if(time1 > 4.0 && time1 < 6.0)
    {
      // arm setup
      kortex_speed[0]=0/10.0; // TODO why divide by 10.0?
      kortex_speed[1]=0/10.0;
      kortex_speed[2]=0/10.0;
      kortex_speed[3]=0/10.0;
      kortex_speed[4]=0/10.0;
      kortex_speed[5]=0/10.0;
      kortex_speed[6]=0/10.0;

      example_send_joint_speeds(n, robot_name, degrees_of_freedom, kortex_speed);

      // move forward slightly, indicating robot is ready
      double Vx = 0.01; // forward speed
      double Vy = 0.0; // sideways speed (strafe)
      double omega = 0.0; // rotation speed
      VectorXd velocities = getWheelVelocities(Vx, Vy, omega);
      VectorXd Speed_wheel = velocities*109/pi;

      // assign messages for publishing
      velKF1.data = -Speed_wheel(1);
      velKF2.data = -Speed_wheel(0);
      velKR1.data = Speed_wheel(2);
      velKR2.data = Speed_wheel(3);

      // publish wheel speeds
      velocityKF1_pub.publish(velKF1);
      velocityKF2_pub.publish(velKF2);
      velocityKR1_pub.publish(velKR1);
      velocityKR2_pub.publish(velKR2);

    }
    else if(time1 > 6.0)
    {
      // regular operation begins
      kortex_speed[0]=0;
      kortex_speed[1]=0;
      kortex_speed[2]=0;
      kortex_speed[3]=0;
      kortex_speed[4]=0;
      kortex_speed[5]=0;
      kortex_speed[6]=0;
      example_send_joint_speeds(n, robot_name, degrees_of_freedom, kortex_speed);

      double Vx = 0.0; // forward speed
      double Vy = 0.0; // sideways speed (strafe)
      double omega = 0.0; // rotation speed

      // process forces (apply offset and compensate for geometry of setup, force sensor mounted on an angle)
      double f_vertical = -0.7071*((end_f[0]-row_means(0))-(end_f[1]-row_means(1))); // x-axis is in 45 and y-axis is in 135 from ground normal - 0.7071 is cos(45)
      double fz = end_f[2] - row_means(2);
      double tau_y = end_tq[1] - row_means(4);

      // define thresholds for force and torque
      const double f_thresh = 5;
      const double t_thresh = 2.5;
      
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
          //Vy = 0.0;
          omega = -0.5;
        }
        else if (tau_y < -t_thresh) // tau_y < -2.5
        {
          Vx = 0.0;
          //Vy = 0.0;
          omega = 0.5;
        }
      }

      // -------------------------------------------------------------------------------------------------------------------------------------------

      // uses a rolling function to calculate final speed
      // this reduces jitter in the system that comes from sudden changes in force
      float ratio = 0.05; // determines what percent of the final speed is dictated by the new velocity/omega value
      Vx_final = (1-ratio)*Vx_final + ratio*Vx;
      Vy_final = (1-ratio)*Vy_final + ratio*Vy;
      omega_final = (1-ratio)*omega_final + ratio*omega;

      VectorXd velocities = getWheelVelocities(Vx_final, Vy_final, omega_final); // calculate individual wheel velocities
      VectorXd Speed_wheel = velocities*109/pi;

      // assign messages for publishing
      velKF1.data = -Speed_wheel(1);
      velKF2.data = -Speed_wheel(0);
      velKR1.data = Speed_wheel(2);
      velKR2.data = Speed_wheel(3);

      // publish wheel speeds
      velocityKF1_pub.publish(velKF1);
      velocityKF2_pub.publish(velKF2);
      velocityKR1_pub.publish(velKR1);
      velocityKR2_pub.publish(velKR2);

      // print raw force values, along with processed force values
      if (count%20 == 0)
      {
        cout << end_f[0] << "  " << end_f[1] << "  " << end_f[2] << "  " << end_tq[0] << "  " << end_tq[1] << "  " << end_tq[2] << " " << f_vertical << " " << fz << " " << tau_y << endl;
      }

      count=count+1;

      // logs all raw and processed force values to a file
      fprintf(fpforce,"%.5f   %.5f   %.5f    %.5f    %.5f    %.5f   %.5f   %.5f   %.5f   %.5f\r\n",time1,end_f[0],end_f[1],end_f[2],end_tq[0],end_tq[1],end_tq[2],f_vertical,fz,tau_y);
	  }
    
    time_old=time_now;

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}