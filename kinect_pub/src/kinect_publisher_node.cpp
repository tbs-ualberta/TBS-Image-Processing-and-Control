// Authored by: Andor Siegers
// RUN BEFORE STARTING ROS PROGRAMS: source ~/catkin_ws/devel/setup.bash
// THEN TO RUN NODE: rosrun kinect_pub kinect_pub_node

// For info on the Kinect V2's FOV see: https://smeenk.com/kinect-field-of-view-comparison/
// For info on the Kinect V2's Camera Parameters see: https://github.com/shanilfernando/VRInteraction/tree/master/calibration
// Note: these will be slightly different on each camera. This program provides a ROS service for accessing the factory calibration
// values stored on the camera, however image registration is also calculated in this program and published to a ROS topic,
// enabling easy conversion between RGB pixel space and Cartesian space.

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/registration.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "kinect_pub/GetCameraInfo.h"
#include "kinect_pub/RegistrationData.h"

// ------------------------------- User defined constants -------------------------------

// Publishing frequency of the program
const int LOOP_RATE = 30; // in Hz

// How long the program waits for images to be sent from the kinect before timing out
const int TIMEOUT = 10; // in seconds

// --------------------------------------------------------------------------------------

libfreenect2::Freenect2Device::IrCameraParams irParams;
libfreenect2::Freenect2Device::ColorCameraParams rgbParams;

bool getCameraInfo(kinect_pub::GetCameraInfo::Request &req,
                   kinect_pub::GetCameraInfo::Response &res) {
    res.ir_fx = irParams.fx;
    res.ir_fy = irParams.fy;
    res.ir_cx = irParams.cx;
    res.ir_cy = irParams.cy;
    res.rgb_fx = rgbParams.fx;
    res.rgb_fy = rgbParams.fy;
    res.rgb_cx = rgbParams.cx;
    res.rgb_cy = rgbParams.cy;
    return true;
}

// To convert registration data
sensor_msgs::ImagePtr frameToImageMsg(const libfreenect2::Frame *frame, const std::string &encoding)
{
    cv::Mat image(frame->height, frame->width, CV_8UC4, frame->data);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), encoding, image).toImageMsg();
    return msg;
}

int main(int argc, char **argv)
{
    // setup ROS node
    ros::init(argc, argv, "kinect2_publisher");
    ros::NodeHandle nh;

    // ROS publishers for RGB and depth images
    ros::Publisher rgb_pub = nh.advertise<sensor_msgs::Image>("/rgbd_out/rgb", 1);
    ros::Publisher depth_raw_pub = nh.advertise<sensor_msgs::Image>("/rgbd_out/depth_raw", 1);
    ros::Publisher depth_norm_pub = nh.advertise<sensor_msgs::Image>("/rgbd_out/depth_norm", 1);
    ros::Publisher ir_raw_pub = nh.advertise<sensor_msgs::Image>("/rgbd_out/ir_raw", 1);
    ros::Publisher ir_norm_pub = nh.advertise<sensor_msgs::Image>("/rgbd_out/ir_norm", 1);
    // For registration images and data
    ros::Publisher reg_data_pub = nh.advertise<kinect_pub::RegistrationData>("/rgbd_out/reg_data", 1);

    // Initialize Kinect
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = nullptr;

    dev = freenect2.openDefaultDevice();

    if (freenect2.enumerateDevices() == 0) {
        ROS_ERROR("No device connected!");
        return -1;
    }

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

    // Get calibration data
    irParams = dev->getIrCameraParams();
    rgbParams = dev->getColorCameraParams();

    // Initialize service to send camera intrinsics
    ros::ServiceServer cam_info_serv = nh.advertiseService("/rgbd_out/get_camera_info", getCameraInfo);
    ROS_INFO("Ready to provide camera info.");

    // For registration; this only initializes the registration object, and does not do any processing
    libfreenect2::Registration* registration = new libfreenect2::Registration(irParams, rgbParams);
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
    libfreenect2::Frame bigdepth(1920, 1080 + 2, 4); // mapping of depth onto colour pixels
    int colour_depth_map [512 * 424]; // index of mapped colour pixel for each depth pixel
    

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

        // Calculate, convert, and publish registered frame
        // Process registered frame
        registration->apply(rgb, depth, &undistorted, &registered, true, &bigdepth, colour_depth_map);

        // Convert to reg data to message types
        cv::Mat reg_image(registered.height, registered.width, CV_8UC4, registered.data);
        sensor_msgs::ImagePtr reg_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgra8", reg_image).toImageMsg();

        cv::Mat undistorted_image(undistorted.height, undistorted.width, CV_32FC1, undistorted.data);
        sensor_msgs::ImagePtr undistorted_msg = cv_bridge::CvImage(std_msgs::Header(), "32FC1", undistorted_image).toImageMsg();
        sensor_msgs::ImagePtr big_depth_msg = frameToImageMsg(&bigdepth, sensor_msgs::image_encodings::TYPE_32FC1);
        

        kinect_pub::RegistrationData registration_data_msg;

        registration_data_msg.rgb_image = *rgb_msg; // RGB image
        registration_data_msg.depth_image = *depth_raw_msg; // Depth image
        registration_data_msg.undistorted_image = *undistorted_msg; // Undistorted depth image
        registration_data_msg.registered_image = *reg_img_msg; // RGB image mapped to depth image
        registration_data_msg.bigdepth_image = *big_depth_msg; // Maps depth onto RGB

        // Fill colour_depth_map
        for (int i = 0; i < 512 * 424; ++i)
        {
            registration_data_msg.colour_depth_map.push_back(colour_depth_map[i]); // Maps RGB onto depth
        }
        
        // Publish registration data
        reg_data_pub.publish(registration_data_msg);


        listener.release(frames);

        ros::spinOnce();
        loop_rate.sleep();
    }

    dev->stop();
    dev->close();

    delete registration; // For registration

    return 0;
}