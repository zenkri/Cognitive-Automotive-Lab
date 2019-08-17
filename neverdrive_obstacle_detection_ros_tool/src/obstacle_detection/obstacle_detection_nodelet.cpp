#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "obstacle_detection.hpp"

namespace neverdrive_obstacle_detection_ros_tool {

class ObstacleDetectionNodelet : public nodelet::Nodelet {

    inline void onInit() override {
        impl_ = std::make_unique<ObstacleDetection>(getNodeHandle(), getPrivateNodeHandle());
    }
    std::unique_ptr<ObstacleDetection> impl_;
};
} // namespace neverdrive_obstacle_detection_ros_tool

PLUGINLIB_EXPORT_CLASS(neverdrive_obstacle_detection_ros_tool::ObstacleDetectionNodelet, nodelet::Nodelet);
