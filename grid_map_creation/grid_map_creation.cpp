#include "grid_map_creation.hpp"

namespace neverdrive_grid_map_ros_tool {

GridMapCreation::GridMapCreation(ros::NodeHandle nhPublic, ros::NodeHandle nhPrivate)
        : interface_{nhPrivate}, reconfigureServer_{nhPrivate}, current_task_{0} {

    /**
     * Initialization
     */
    interface_.fromParamServer();

    /**
     * Set up callbacks for subscribers and reconfigure.
     *
     * New subscribers can be created with "add_subscriber" in "cfg/GridMapCreation.if file.
     * Don't forget to register your callbacks here!
     */
    reconfigureServer_.setCallback(boost::bind(&GridMapCreation::reconfigureRequest, this, _1, _2));

    current_task_subscriber_ = nhPrivate.subscribe("/current_task",
                                                   1,
                                                   &GridMapCreation::setTaskCallback,
                                                   this);

    point_cloud2_subscriber_ = nhPrivate.subscribe("/traffic_cones_point_cloud",
                                               1,
                                               &GridMapCreation::pointCloudCallback,
                                               this);
    grid_map_publisher_ = nhPrivate.advertise<nav_msgs::OccupancyGrid>("/grid_map", 1);

    map_.header.frame_id="world";
    map_.info.height = interface_.map_height / interface_.resolution;
    map_.info.width = interface_.map_width / interface_.resolution;
    map_.info.resolution = interface_.resolution;
    geometry_msgs::Pose pose_map;
    pose_map.position.x = interface_.origin_x;
    pose_map.position.y = interface_.origin_y;
    pose_map.position.z = 0.0;
    pose_map.orientation.x = 0.0;
    pose_map.orientation.y = 0.0;
    pose_map.orientation.z = 0.0;
    pose_map.orientation.w = 1.0;
    map_.info.origin=pose_map;
    map_.data = std::vector<int8_t>(map_.info.width * map_.info.height, 50);

    rosinterface_handler::showNodeInfo();
}

/**
  * This callback is called at startup or whenever a change was made in the dynamic_reconfigure window
*/
void GridMapCreation::reconfigureRequest(const Interface::Config& config, uint32_t level) {
    interface_.fromConfig(config);
}

void GridMapCreation::setTaskCallback(std_msgs::Int8 msg){
    current_task_=msg.data;
}

void GridMapCreation::pointCloudCallback(const sensor_msgs::PointCloud2 &cloud_msg){
    if(current_task_!=2 && current_task_!=3){ return;}

    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(cloud_msg, cloud);

    Eigen::MatrixXi update_mask= Eigen::MatrixXi::Zero(map_.info.height, map_.info.width);
    Eigen::MatrixXi update_mask_pylon = Eigen::MatrixXi::Zero(map_.info.height, map_.info.width);
    Eigen::MatrixXi update_mask_ground = Eigen::MatrixXi::Zero(map_.info.height, map_.info.width);
    Eigen::MatrixXd update_value = Eigen::MatrixXd::Ones(map_.info.height, map_.info.width) *
                                   (100.0 - interface_.model_confidence);
    for (auto& point : cloud.points) {

        if (point.y < map_.info.origin.position.y || point.x < map_.info.origin.position.x) {
            continue;
        }

        auto index_row = static_cast<size_t>((point.y - map_.info.origin.position.y) / map_.info.resolution);
        auto index_col = static_cast<size_t>((point.x - map_.info.origin.position.x) / map_.info.resolution);

        if (index_row >= map_.info.height || index_col >= map_.info.width) {
            continue;
        }

        ++update_mask(index_row, index_col);
        if (point.intensity == 0) {
        update_value(index_row, index_col) += interface_.weight_per_point;
        update_value(index_row, index_col) = update_value(index_row, index_col) > 99.0 ? 99.0 : update_value(index_row, index_col);
        }

        /*else{
            update_value(index_row, index_col) -= interface_.weight_per_point*10;
            update_value(index_row, index_col) = update_value(index_row, index_col) < 1.0 ? 1.0 : update_value(index_row, index_col);

        }

        if (point.intensity == 0){
            update_mask_pylon(index_row, index_col);
        } else{
            update_mask_ground(index_row, index_col);
        }
        */

    }

    for (size_t i = 0; i < map_.info.height; i++)
        for (size_t j = 0; j < map_.info.width; j++) {
            if (update_mask(i, j) < interface_.update_threshold) { continue; }

            auto old_value = static_cast<double>(map_.data[i * map_.info.width + j]);
	    if(old_value > 70){continue;}

            double s1 = old_value / (100.0 - old_value);
            double s2 = update_value(i, j) / ( 100.0 - update_value(i, j) );

            double s = s1 * s2;
            auto new_value = static_cast<int8_t>(100 * s / (s + 1));

            new_value = new_value > 99 ? 99 : new_value;
            new_value = new_value < 1 ? 1 : new_value;
            map_.data[i * map_.info.width + j] = new_value;
        }


    nav_msgs::OccupancyGrid map_pub = map_;
    for (size_t i = 0; i < map_pub.info.height; i++)
        for (size_t j = 0; j < map_pub.info.width; j++) {
            if( map_pub.data[i * map_pub.info.width + j] < 51 )
            { map_pub.data[i * map_pub.info.width + j] = 0; }
        }
    grid_map_publisher_.publish(map_);


}



} // namespace neverdrive_grid_map_ros_tool
