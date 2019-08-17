#include "pedestrian_detection.hpp"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "pedestrian_detection_node");

    pedestrian_detection_ros_tool::PedestrianDetection pedestrian_detection(ros::NodeHandle(), ros::NodeHandle("~"));

    ros::spin();
    return EXIT_SUCCESS;
}
