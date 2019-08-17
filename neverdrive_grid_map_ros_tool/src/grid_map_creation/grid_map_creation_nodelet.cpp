#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "grid_map_creation.hpp"

namespace neverdrive_grid_map_ros_tool {

class GridMapCreationNodelet : public nodelet::Nodelet {

    inline void onInit() override {
        impl_ = std::make_unique<GridMapCreation>(getNodeHandle(), getPrivateNodeHandle());
    }
    std::unique_ptr<GridMapCreation> impl_;
};
} // namespace neverdrive_grid_map_ros_tool

PLUGINLIB_EXPORT_CLASS(neverdrive_grid_map_ros_tool::GridMapCreationNodelet, nodelet::Nodelet);
