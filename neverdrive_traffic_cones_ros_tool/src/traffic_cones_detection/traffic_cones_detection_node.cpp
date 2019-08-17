#include "traffic_cones_detection.hpp"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "traffic_cones_detection_node");

    neverdrive_traffic_cones_ros_tool::TrafficConesDetection traffic_cones_detection(ros::NodeHandle(),
                                                                                     ros::NodeHandle("~"));

    ros::spin();
    return EXIT_SUCCESS;
}
