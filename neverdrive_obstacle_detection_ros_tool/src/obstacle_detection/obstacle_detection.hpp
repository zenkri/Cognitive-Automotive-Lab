#pragma once

#include <dynamic_reconfigure/server.h>
#include <ros/forwards.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <opencv/cv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>

#include "neverdrive_obstacle_detection_ros_tool/ObstacleDetectionInterface.h"
#include "neverdrive_obstacle_detection_ros_tool/Box.h"

using namespace std;
using namespace cv;
using namespace ros;

namespace neverdrive_obstacle_detection_ros_tool {

class ObstacleDetection {

    using Interface = ObstacleDetectionInterface;

    using Msg = sensor_msgs::Image;

public:
    ObstacleDetection(ros::NodeHandle, ros::NodeHandle);

private:
    void callbackTimer(const ros::TimerEvent&);
    void callbackSubscriber(const Msg::ConstPtr& msg);
    void reconfigureRequest(const Interface::Config&, uint32_t);
    /*****/

    void maskDepth(Mat & ground, Mat &, Mat &, int, int );
    void findObstacleContour(Mat &image, int minThre , int maxThre , int area ,int maskSize );
    bool pixelInMapArea(cv::Mat & image,int v,int u);
    void getIr2World();

    pcl::PointXYZ pointInCamera (int pixelU, int pixelV, double depth);
    double getObstacleDistance(Mat &depthImage, Rect contourRect);
    bool isObstacleAhead(Rect rect, double depth);
    void depthImageCallback(const Msg::ConstPtr& msg);
    void colorImageCallback(const Msg::ConstPtr& msg);
    void taskCallback(const std_msgs::Int8ConstPtr& msg);
    void depthImageCallback(const ros::TimerEvent&);
    void readGroundMap(const std::string&);


    cv::Mat ground_depth_;
    Eigen::Affine3d ir_2_world_;
    ros::Subscriber depth_image_subscriber_;
    ros::Subscriber color_image_subscriber_;
    ros::Publisher depth_image_publisher_;
    ros::Publisher color_image_publisher_;
    ros::Publisher obstacle_detected_publisher_;
    ros::Timer depth_image_pub_timer_;
    ros::Subscriber current_task_subscriber_;

    int8_t current_task_ =0;


    Rect boxOfObstacle;
    double timerecoder_;
    int count_ =0;
    const int ground_threshold_ = 100;
    bool changeThePath_{false};
    bool isObstacles{false};
    bool in_reset_{false};

    neverdrive_obstacle_detection_ros_tool::Box box_detected_msg;

    /*****/
    Interface interface_;
    dynamic_reconfigure::Server<Interface::Config> reconfigureServer_;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_{tfBuffer_};
    tf2_ros::TransformBroadcaster tfBroadcaster_;
};
} // namespace neverdrive_obstacle_detection_ros_tool
