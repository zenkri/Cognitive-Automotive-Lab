#pragma once

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <dynamic_reconfigure/server.h>
#include <ros/forwards.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Eigen>

#include <std_msgs/Int8.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud.h>

#include "neverdrive_grid_map_ros_tool/GridMapCreationInterface.h"

namespace neverdrive_grid_map_ros_tool {

class GridMapCreation {

    using Interface = GridMapCreationInterface;

    using Msg = std_msgs::Header;

public:
    GridMapCreation(ros::NodeHandle, ros::NodeHandle);

private:
    void callbackSubscriber(const Msg::ConstPtr& msg);
    void reconfigureRequest(const Interface::Config&, uint32_t);

    /***************/
    void setTaskCallback(std_msgs::Int8 msg);
    void pointCloudCallback(const sensor_msgs::PointCloud2 &cloud);


    int8_t current_task_;
    ros::Subscriber current_task_subscriber_;
    ros::Subscriber point_cloud2_subscriber_;
    ros::Publisher grid_map_publisher_;
    nav_msgs::OccupancyGrid map_;

    /***************/

    Interface interface_;
    dynamic_reconfigure::Server<Interface::Config> reconfigureServer_;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_{tfBuffer_};
    tf2_ros::TransformBroadcaster tfBroadcaster_;
};
} // namespace neverdrive_grid_map_ros_tool
