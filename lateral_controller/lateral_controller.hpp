#pragma once

#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Path.h>

#include <Eigen/Dense>

#include "anicar_lateral_control_ros_tool/LateralControllerParameters.h"
#include "motor_interface_ros_tool/MotorCommand.h"

#include <std_msgs/Int8.h>

namespace anicar_lateral_control_ros_tool {

class LateralController {

    using Parameters = LateralControllerParameters;
    using Config = LateralControllerConfig;
    using ReconfigureServer = dynamic_reconfigure::Server<Config>;
    using MotorCommand = motor_interface_ros_tool::MotorCommand;

public:
    LateralController(ros::NodeHandle, ros::NodeHandle);

private:
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);
    void controlLoopCallback(const ros::TimerEvent& timer_event);
    void velocityCallback(const MotorCommand::ConstPtr& msg);
    void unknowenvironmentCallback(const std_msgs::Int8ConstPtr &msg);
    void reconfigureRequest(const Config&, uint32_t);

    std::vector<Eigen::Affine3d> path_;

    ros::Publisher servo_command_publisher_;
    ros::Publisher zebra_cross_publisher_;
    ros::Subscriber path_subscriber_;
    ros::Subscriber velocity_subscriber_;
    ros::Subscriber unknow_envrionmet_subscriber;
    ros::Timer control_loop_timer_;
    Parameters params_;
    ReconfigureServer reconfigureServer_;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    tf2_ros::TransformBroadcaster tfBroadcaster_;

    bool *judgment;
    double curr_velo;
    double err_last=0, angle_last=0, err_integral=0;
    int8_t task_param = 0;
    std::string sign_flag;
    std_msgs::String zebra_cross_msg;
};
} // namespace anicar_lateral_control_ros_tool
