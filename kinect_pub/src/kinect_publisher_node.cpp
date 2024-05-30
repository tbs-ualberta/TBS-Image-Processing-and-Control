// Authored by: Andor Siegers
// RUN BEFORE STARTING ROS PROGRAMS: source ~/catkin_ws/devel/setup.bash
// THEN TO RUN NODE: rosrun kinect_pub kinect_pub_node

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/registration.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// ------------------------------- User defined constants -------------------------------

// Publishing frequency of the program
const int LOOP_RATE = 30; // in Hz

// How long the program waits for images to be sent from the kinect before timing out
const int TIMEOUT = 10; // in seconds

// --------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    // setup ROS node
    ros::init(argc, argv, "kinect2_publisher");
    ros::NodeHandle nh;

    // ROS publishers for RGB and depth images
    ros::Publisher rgb_pub = nh.advertise<sensor_msgs::Image>("/kinect2/rgb", 1);
    ros::Publisher depth_raw_pub = nh.advertise<sensor_msgs::Image>("/kinect2/depth_raw", 1);
    ros::Publisher depth_norm_pub = nh.advertise<sensor_msgs::Image>("/kinect2/depth_norm", 1);
    ros::Publisher ir_raw_pub = nh.advertise<sensor_msgs::Image>("/kinect2/ir_raw", 1);
    ros::Publisher ir_norm_pub = nh.advertise<sensor_msgs::Image>("/kinect2/ir_norm", 1);
    ros::Publisher reg_pub = nh.advertise<sensor_msgs::Image>("/kinect2/reg", 1); // For registration

    // Initialize Kinect
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = 0;

    dev = freenect2.openDefaultDevice();

    if (dev == nullptr) {
        ROS_ERROR("Failed to open Kinect device!");
        return -1;
    }

    // Set up listeners for RGB and depth frames
    libfreenect2::SyncMultiFrameListener listener(
        libfreenect2::Frame::Color | libfreenect2::Frame::Depth | libfreenect2::Frame::Ir);

    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);

    // Start kinect
    if(!dev->start())
        return -1;

    ROS_INFO("Kinect device started.");
    std::string srl_msg = "Device serial: " + dev->getSerialNumber();
    std::string frmwr_msg = "Device firmware: " + dev->getFirmwareVersion();
    ROS_INFO(srl_msg.c_str());
    ROS_INFO(frmwr_msg.c_str());

    // For registration; this only initializes the registration object, and does not do any processing
    libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
    libfreenect2::Frame bigdepth(1920, 1080 + 2, 4);
    int colour_depth_ind [512 * 424];
    

    ros::Rate loop_rate(LOOP_RATE);

    while (ros::ok()) {
        libfreenect2::FrameMap frames;
        if (!listener.waitForNewFrame(frames, TIMEOUT * 1000)) { // timeout
            ROS_WARN("Timeout while waiting for new Kinect frames.");
            continue;
        }

        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];        
        libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];

        // Process depth and IR frames
        // Normalize depth frame
        cv::Mat depth_image(depth->height, depth->width, CV_32FC1, depth->data);
        cv::Mat depth_normalized;
        cv::normalize(depth_image, depth_normalized, 0, 1, cv::NORM_MINMAX);

        // Normalize IR frame
        cv::Mat ir_image(ir->height, ir->width, CV_32FC1, ir->data);
        cv::Mat ir_normalized;
        cv::normalize(ir_image, ir_normalized, 0, 1, cv::NORM_MINMAX);


        // Publish frames
        // Convert RGB frame to ROS message and publish
        cv::Mat rgb_image(rgb->height, rgb->width, CV_8UC4, rgb->data);
        sensor_msgs::ImagePtr rgb_msg = cv_bridge::CvImage(std_msgs::Header(), "bgra8", rgb_image).toImageMsg();
        rgb_pub.publish(rgb_msg);

        // Convert raw depth frame to ROS message and publish
        sensor_msgs::ImagePtr depth_raw_msg = cv_bridge::CvImage(std_msgs::Header(), "32FC1", depth_image).toImageMsg();
        depth_raw_pub.publish(depth_raw_msg);

        // Convert normalized depth frame to ROS message and publish
        sensor_msgs::ImagePtr depth_norm_msg = cv_bridge::CvImage(std_msgs::Header(), "32FC1", depth_normalized).toImageMsg();
        depth_norm_pub.publish(depth_norm_msg);

        // Convert raw IR frame to ROS message and publish
        sensor_msgs::ImagePtr ir_raw_msg = cv_bridge::CvImage(std_msgs::Header(), "32FC1", ir_image).toImageMsg();
        ir_raw_pub.publish(ir_raw_msg);

        // Convert normalized IR frame to ROS message and publish
        sensor_msgs::ImagePtr ir_norm_msg = cv_bridge::CvImage(std_msgs::Header(), "32FC1", ir_normalized).toImageMsg();
        ir_norm_pub.publish(ir_norm_msg);


        // Process registered frame
        registration->apply(rgb, depth, &undistorted, &registered, true, &bigdepth, colour_depth_ind);

        // Convert registered frame to ROS message and publish
        cv::Mat reg_image(registered.height, registered.width, CV_8UC4, registered.data);
        sensor_msgs::ImagePtr reg_msg = cv_bridge::CvImage(std_msgs::Header(), "bgra8", reg_image).toImageMsg();
        reg_pub.publish(reg_msg);
        
        
        listener.release(frames);

        ros::spinOnce();
        loop_rate.sleep();
    }

    dev->stop();
    dev->close();

    delete registration; // For registration

    return 0;
}