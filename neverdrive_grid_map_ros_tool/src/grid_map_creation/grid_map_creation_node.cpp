#include "grid_map_creation.hpp"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "grid_map_creation_node");

    neverdrive_grid_map_ros_tool::GridMapCreation grid_map_creation(ros::NodeHandle(), ros::NodeHandle("~"));

    ros::spin();
    return EXIT_SUCCESS;
}
