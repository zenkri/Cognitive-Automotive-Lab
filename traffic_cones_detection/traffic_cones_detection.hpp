#pragma once

#include <dynamic_reconfigure/server.h>
#include <ros/forwards.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <queue>

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <opencv/cv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <std_msgs/Int8.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>

#include "neverdrive_traffic_cones_ros_tool/TrafficConesDetectionInterface.h"

using namespace std;
using namespace cv;
namespace neverdrive_traffic_cones_ros_tool {

using Img = sensor_msgs::Image;
using SynchronizedKinectImages = message_filters::sync_policies::ApproximateTime<Img, Img>;

class TrafficConesDetection {

    using Interface = TrafficConesDetectionInterface;

    using Msg = std_msgs::Header;

public:
    TrafficConesDetection(ros::NodeHandle, ros::NodeHandle);

private:
    void reconfigureRequest(const Interface::Config&, uint32_t);

    /***********************/
    void setTaskCallback(std_msgs::Int8 msg);
    void setupPublisher(ros::NodeHandle&);
    void setupSubscriber(ros::NodeHandle&);
    void setTaskSubscriber(ros::NodeHandle&);
    void setupIntrinsics(double resize_scale);
    void callbackKinectImages(const Img::ConstPtr& image, const Img::ConstPtr& depth);
    void filterImage(cv::Mat& , cv::Mat&);
    void shiftImage(const cv::Mat&, const cv::Mat&, cv::Mat&, const double);
    void getIr2World();
    void project2PointCloud(const cv::Mat& image, const cv::Mat& depth, const Eigen::Affine3d& transform, const pcl::PointCloud<pcl::PointXYZI>::Ptr& points);

    int8_t current_task_;
    Eigen::Affine3d ir_2_world_;
    cv::Mat structuring_element_0_;
    double resized_focal_length_, resized_u0_, resized_v0_;
    std::unique_ptr<message_filters::Subscriber<Img>> subscriber_image_;
    std::unique_ptr<message_filters::Subscriber<Img>> subscriber_depth_;
    std::unique_ptr<message_filters::Synchronizer<SynchronizedKinectImages>> kinect_image_synchronizer_;

    ros::Subscriber current_task_subscriber_;
    ros::Publisher publisher_filtered_image_;
    ros::Publisher publisher_morph_open_image_;
    ros::Publisher publisher_point_cloud_;
    ros::Publisher publisher_original_depth_;
    /************************/

    Interface interface_;
    dynamic_reconfigure::Server<Interface::Config> reconfigureServer_;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_{tfBuffer_};
    tf2_ros::TransformBroadcaster tfBroadcaster_;
};
} // namespace neverdrive_traffic_cones_ros_tool
