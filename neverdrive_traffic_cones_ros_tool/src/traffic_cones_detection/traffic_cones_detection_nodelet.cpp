#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "traffic_cones_detection.hpp"

namespace neverdrive_traffic_cones_ros_tool {

class TrafficConesDetectionNodelet : public nodelet::Nodelet {

    inline void onInit() override {
        impl_ = std::make_unique<TrafficConesDetection>(getNodeHandle(), getPrivateNodeHandle());
    }
    std::unique_ptr<TrafficConesDetection> impl_;
};
} // namespace neverdrive_traffic_cones_ros_tool

PLUGINLIB_EXPORT_CLASS(neverdrive_traffic_cones_ros_tool::TrafficConesDetectionNodelet, nodelet::Nodelet);
