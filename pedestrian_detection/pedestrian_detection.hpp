#pragma once

#include <dynamic_reconfigure/server.h>
#include <ros/forwards.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "pedestrian_detection_ros_tool/PedestrianDetectionInterface.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <limits>
#include <opencv2/video/tracking.hpp>

//#include <tensorflow/core/framework/tensor.h>
#include "pedestrian_detector/pedestrian_classifier.h"
#include <std_msgs/Header.h>
#include <Python.h>

namespace pedestrian_detection_ros_tool {

class PedestrianDetection {

    using Interface = PedestrianDetectionInterface;

    using Classifier = pedestrian_detector::pedestrian_classifier;

    using Msg = std_msgs::Header;

public:
    PedestrianDetection(ros::NodeHandle, ros::NodeHandle);

private:
    void depthimageSubscriberCallback(const sensor_msgs::ImageConstPtr &msg);
    void colorimageSubscriberCallback(const sensor_msgs::ImageConstPtr &msg);
    void zebracrossCallback(const std_msgs::StringConstPtr &msg);
    void publishTimerCallback(const ros::TimerEvent &timer_event);
    void reconfigureRequest(const Interface::Config&, uint32_t);
    Eigen::Vector3d worldXYZ(const cv::Mat image, const double u, const double v);

    struct Carmera_Info{
    	double camera_factor=1000;
    	double cy=269.75,cx=479.75,fx=540.686,fy=540.686;
    };

    Carmera_Info camera_info;

    Interface interface_;
    dynamic_reconfigure::Server<Interface::Config> reconfigureServer_;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_{tfBuffer_};
    tf2_ros::TransformBroadcaster tfBroadcaster_;

    ros::Subscriber depth_image_subscriber;
    ros::Subscriber color_image_subscriber;
    ros::Subscriber zebra_cross_subscriber;
    ros::Publisher label_color_image_publisher;
    ros::Publisher label_depth_image_publisher;
    ros::Publisher pedestrian_detector_publisher;
    ros::Timer publish_loop_timer;
    ros::ServiceClient roi_classifier;

    std_msgs::Header header;
    sensor_msgs::ImagePtr color_label_image_msg,depth_label_image_msg;

    cv::Mat depth_image,color_image;
    Eigen::Affine3d camera_to_world;
    std::string command_zebra_cross;

    int output_image_counter=0;

    cv::KalmanFilter robot_track;
    bool first_kalman = true;
    std_msgs::String pedestrian_msg;
    cv::Mat prediction,estimated;
    cv::Mat_<float> measurement;
    float estimated_v = 0;
    ros::Time see_robot;

    Classifier srv;

};
} // namespace pedestrian_detection_ros_tool
