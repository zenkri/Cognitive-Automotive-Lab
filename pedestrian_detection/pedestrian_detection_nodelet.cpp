#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "pedestrian_detection.hpp"

namespace pedestrian_detection_ros_tool {

class PedestrianDetectionNodelet : public nodelet::Nodelet {

    inline void onInit() override {
        impl_ = std::make_unique<PedestrianDetection>(getNodeHandle(), getPrivateNodeHandle());
    }
    std::unique_ptr<PedestrianDetection> impl_;
};
} // namespace pedestrian_detection_ros_tool

PLUGINLIB_EXPORT_CLASS(pedestrian_detection_ros_tool::PedestrianDetectionNodelet, nodelet::Nodelet);
