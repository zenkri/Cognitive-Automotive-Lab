#pragma once

#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>


#include "anicar_longitudinal_control_ros_tool/LongitudinalControllerParameters.h"

namespace anicar_longitudinal_control_ros_tool {

class LongitudinalController {

    using Parameters = LongitudinalControllerParameters;
    using Config = LongitudinalControllerConfig;
    using ReconfigureServer = dynamic_reconfigure::Server<Config>;

public:
    LongitudinalController(ros::NodeHandle, ros::NodeHandle);

private:
    void controlLoopCallback(const ros::TimerEvent& timer_event);
    void reconfigureRequest(const Config&, uint32_t);
    void getStopCallback(const std_msgs::String::Ptr& msg);
    void pedestrianMsgCallback(const std_msgs::StringConstPtr &msg);
    void taskparameterCallback(const std_msgs::Int8ConstPtr &msg);


    ros::Subscriber stop_subscriber_;
    bool already_see_stop_=false,already_brake_stop_sign=false;
    std::string stop_or_drive_;
    ros::Time stop_time_stop_sign,brake_time_;
    double cos_param;

    ros::Publisher motor_command_publisher_;
    ros::Timer control_loop_timer_;
    Parameters params_;

    ros::Subscriber pedestrian_subscriber;
    std::string pedestrian_command;
    bool already_see_robot=false,already_see_go = true,already_brake_robot=false;
    ros::Time stop_time_robot;

    int8_t task_param;
    ros::Subscriber task_parameter_subscriber;
    bool already_brake_box = false, already_brake_unknow_environment = false;
    ros::Time task0_time;

    ReconfigureServer reconfigureServer_;
};
} // namespace anicar_longitudinal_control_ros_tool
