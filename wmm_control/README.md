# Introduction
This package contains the main control program for the Wheeled Mobile Manipulator (WMM). It contains the original control program (Test_base.cpp) as well as an updated program that supports the XBox Kinect integration. This allows the image processing to effect how the robot is controlled.

# Assumptions
- Operating system: Ubuntu 20.04.6 LTS
- ROS version: Noetic

# Setup
To run this program, a few things must first be done. Launch files assist with this, but some manual setup is also required.
Note: some of this is setup specific and may not be relevant to other setups. This document assumes that one computer is connected to the Kinova arm, Arduino, and force sensor, and runs their corresponding driver programs, while another computer on the same ROS network runs the processing and control programs, as a powerful GPU is required to run the object detection algorithm in near to real time. "roscore" is run on the first computer (connected directly to the robot). To setup a similar network, the instructions given [here](https://wiki.ros.org/ROS/Tutorials/MultipleMachines) are helpful.

## Turn on the Hardware
There are 5 separate components on the robot that need to be physically turned on before proceeding:
1. The Kinova Gen3 Arm - is turned on by holding down the button on the base of the arm for at least 2 seconds (until the light starts to blink).
2. The Arduino - is turned on by plugging in the external 9v battery to the board.
    - Note: after plugging in the battery, two green LEDs should light up on the Arduino. If these do not both light up, try changing the battery.
3. The Mobile Base - is powered on by simply plugging the grey cable into the power socket.
4. The Force Sensor (optional) - is turned on by powering on the connected external power supply.
5. The Xbox Kinect V2 Camera (optional) - is turned on by plugging the black barrel connector into the USB adapter.
The Arduino and Kinova arm then need to be connected to the computer. In our lab, this is done by connecting ethernet1 to the Arduino, while ethernet2 is connected to the Kinova arm.
The Xbox KinectV2 Camera is connected via USB directly to the computer running image processing, although due to the modular nature of ROS, it could be connected to the control computer (although the node would then have to be launched manually, or the launch files would need to be modified). For more information see the "kinect_pub" folder.

## Robot Driver Setup
Once the hardware is turned on, the supporting programs can be started. This is done by running the "robot_startup.launch" file on the computer connected directly to the WMM using the following command:

    roslaunch wmm_control robot_startup.launch

This file launches the Kinova arm driver, the Arduino board controller, and the force sensor driver by default. If for any reason one (or more) of these nodes should not be launched, the "run_arm_driver", "run_ati_sensor", and "run_robot_base" can be set in the file itself, or when launching. For example:    

    roslaunch wmm_control robot_startup.launch run_arm_driver:=true run_ati_sensor:=false run_wheeled_robot:=true

## Running the Control Program
Once the support programs are running, the control program, along with the object recognition and it's supporting programs can be run. This can be done using the "test_control.launch" launch file. This is separated from the first launch file so that it can be run on a different machine, as it requires a powerful GPU that the computer driving the robot may not have. It can be run using the following command:

    roslaunch wmm_control test_control.launch

It also contains parameters to adjust which nodes launch. For more information about each node and the specific parameter names, see the launch file. The nodes launched are included in the kinect_pub and object_detection packages. See their README files for more information.

# The Control Program
## Function
The control program commands the robot on a high level, first running through initialization steps and then entering the main control loop. Currently, the movement of the robot is controlled using input from the object detection program, which sends the control algorithm the location of the target in relation to the robot. It then utilizes two simple PID controllers to track and follow the specified target. The specifics of the control algorithm are described in detail in the code. Currently, the force sensor data is not used, however the integration of user input (aka the force sensor data) for shared control of the smart walker is a future goal of this project.

## Structure
There are 3 program files in this package:
1. MainController.cpp - contains the main control loop, provides a high level overview of the control algorithm
2. ControlHelper.cpp - contains helper functions that are used in MainController.cpp, this file improves code readability and program modularity
3. Test_base.cpp - contains the previous control function that was used to control the robot using only force data. The first two programs are based off this file.

## Parameters
Control parameters, such as max and min linear and angular velocities, following distance, and the PID parameters can be adjusted at the top of the "MainController.cpp" program file. Physical parameters, such as the wheelbase width and length, and the wheel radius, can be adjusted in the "HelperFunctions.h" header file, which is included in both "MainController.cpp" and "ControlHelper.cpp".
