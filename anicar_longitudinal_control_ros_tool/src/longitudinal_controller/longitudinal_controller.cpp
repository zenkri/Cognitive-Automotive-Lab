#include "longitudinal_controller.hpp"
#include <utils_ros/ros_console.hpp>

#include "motor_interface_ros_tool/MotorCommand.h"

#include <cmath>

using namespace std;

#define PI 3.1415926535898

namespace anicar_longitudinal_control_ros_tool {

using MotorCommand = motor_interface_ros_tool::MotorCommand;

LongitudinalController::LongitudinalController(ros::NodeHandle nh_public, ros::NodeHandle nh_private)
        : params_{nh_private}, reconfigureServer_{nh_private}, already_see_stop_{false}, stop_or_drive_{"DRIVE"} {

    /**
     * Initialization
     */
    utils_ros::setLoggerLevel(nh_private);
    params_.fromParamServer();

    /**
     * Publishers & subscriber
     */
    stop_subscriber_ = nh_private.subscribe("/StopSignDetector",
                                               1,
                                               &LongitudinalController::getStopCallback,
                                               this);
    pedestrian_subscriber = nh_private.subscribe("/pedestrian_detector",
                                                   50,
                                                   &LongitudinalController::pedestrianMsgCallback,
                                                   this);
    task_parameter_subscriber = nh_private.subscribe("/current_task",
            										1,
													&LongitudinalController::taskparameterCallback,
													this);

    motor_command_publisher_ = nh_private.advertise<MotorCommand>(params_.motor_command_topic, params_.msg_queue_size);

    // Instantiate subscriber last, to assure all objects are initialised when first message is received.
    control_loop_timer_ =
        nh_private.createTimer(ros::Rate(params_.control_loop_rate), &LongitudinalController::controlLoopCallback, this);

    /**
     * Set up dynamic reconfiguration
     */
    reconfigureServer_.setCallback(boost::bind(&LongitudinalController::reconfigureRequest, this, _1, _2));

    utils_ros::showNodeInfo();
}

void LongitudinalController::taskparameterCallback(const std_msgs::Int8ConstPtr &msg){
	task_param = msg->data;
}

void LongitudinalController::pedestrianMsgCallback(const std_msgs::StringConstPtr &msg){
	pedestrian_command = msg->data;
}

void LongitudinalController::getStopCallback(const std_msgs::String::Ptr& msg){
    stop_or_drive_=msg->data;
//    ROS_INFO_STREAM("stop or drive: "<< stop_or_drive_ <<endl);
}


void LongitudinalController::controlLoopCallback(const ros::TimerEvent& timer_event) {
    MotorCommand motor_command;
/*
    if((stop_or_drive_=="DRIVE")&&(ros::Time::now().toSec() - stop_time_.toSec() >= params_.wait_time)){
        already_stop_ = false;
        motor_command.velocity = params_.velocity;
        motor_command.sign_flag = "DRIVE";
    }
    if((stop_or_drive_=="STOP") && (already_stop_== false)){
        stop_time_ = ros::Time::now();
        already_stop_ = true;
        motor_command.sign_flag = "STOP";
    } 
    if(already_stop_== true){
        motor_command.header.stamp = timer_event.current_expected;
        if (ros::Time::now().toSec() - stop_time_.toSec() < params_.brake_time){
            motor_command.velocity = -params_.velocity;
        }else if(ros::Time::now().toSec() - stop_time_.toSec() < params_.wait_time){
            motor_command.velocity = 0.0;
        }else{
            motor_command.velocity = params_.velocity;
            motor_command.sign_flag = "DRIVE";
        }
    }else{
    	motor_command.velocity = params_.velocity;
    	motor_command.sign_flag = "DRIVE";
    }
*/
    // cos function to decrease speed

    if (pedestrian_command != "ROBOT"){
    	/*
    	 * Stop sign control part
    	 */

    	if((stop_or_drive_=="DRIVE")&&(ros::Time::now().toSec() - stop_time_stop_sign.toSec() >= params_.wait_time_stop_sign)&&(already_brake_stop_sign==true)){
    		already_see_stop_ = false;
    		already_brake_stop_sign= false;
    		motor_command.velocity = params_.velocity;
    	}
    	if((stop_or_drive_=="STOP") && (already_see_stop_== false)){
    		brake_time_ = ros::Time::now();
    		already_see_stop_ = true;
    	}
    	if(already_see_stop_== true){
    		if (already_brake_stop_sign == false){
    			cos_param=params_.frequence*PI*(ros::Time::now().toSec() - brake_time_.toSec());
    			if(cos_param < PI){
    				motor_command.velocity = params_.velocity*(std::cos(cos_param)+1)/2;
    			}else{
    				motor_command.velocity = 0.0;
    				stop_time_stop_sign=ros::Time::now();
    				already_brake_stop_sign=true;
    			}
    		}else{
    			if (ros::Time::now().toSec() - stop_time_stop_sign.toSec() < params_.wait_time_stop_sign)
    				motor_command.velocity = 0.0;
    			else
    				motor_command.velocity = params_.velocity;
    		}
    	}else{
    		motor_command.velocity = params_.velocity;
    	}

    	/*
    	 * Pedestrian detection control part
    	 */

    	if (already_see_go == false){
    		stop_time_robot = ros::Time::now();
    		already_see_go =true;
    	}
    	if ((already_see_go == true)&&(ros::Time::now().toSec() - stop_time_robot.toSec() > params_.wait_time_robot)){
    		already_see_robot = false;
    		already_brake_robot = false;
    	}
    }else{
    	already_see_go = false;
    	if (already_see_robot == false){
    		already_see_robot = true;
    		brake_time_ = ros::Time::now();
    	}
    	if (already_see_robot == true){
    		if (already_brake_robot == false){
    		    cos_param=params_.frequence*PI*(ros::Time::now().toSec() - brake_time_.toSec());
    		    if(cos_param < PI){
    		    	motor_command.velocity = params_.velocity*(std::cos(cos_param)+1)/2;
    		    }else{
    		    	motor_command.velocity = 0.0;
    		    	already_brake_robot=true;
    		    }
    		}else
    			motor_command.velocity = 0.0;
    	}
    }
    /*
     * decrease velocity when car meets box or into unknown environment
     * because till this step we have already get the velocity information so if we don't change it, it will keep and be sent.
     * thus, if we change task to 0 the velocity will back to max automatic
     */

    if (task_param == 0){
    	already_brake_box = false;
    	already_brake_unknow_environment = false;
    	task0_time = ros::Time::now();
    }else if (task_param == 1){
    	if (already_brake_box == false){
    		cos_param=params_.frequence*PI*(ros::Time::now().toSec() - task0_time.toSec());
    		if (cos_param < PI)
    			motor_command.velocity = (params_.velocity - params_.velocity_box)/2 * (std::cos(cos_param)+1) + params_.velocity_box;
    		else{
    			motor_command.velocity = params_.velocity_box;
    			already_brake_box = true;
    		}
    	}else
    		motor_command.velocity = params_.velocity_box;
    }else if (task_param == 3){
    	if (already_brake_unknow_environment == false){
    		cos_param=params_.frequence*PI*(ros::Time::now().toSec() - task0_time.toSec());
    		if (cos_param < PI)
    			motor_command.velocity = (params_.velocity - params_.velocity_unknow_envrionment)/2 * (std::cos(cos_param)+1) + params_.velocity_unknow_envrionment;
    		else{
    			motor_command.velocity = params_.velocity_unknow_envrionment;
    			already_brake_unknow_environment = true;
    		}
    	}else
    		motor_command.velocity = params_.velocity_unknow_envrionment;
    }


    // Velocity Publish
    motor_command.header.stamp = timer_event.current_expected;
    motor_command_publisher_.publish(motor_command);

}

/**
  * This callback is called at startup or whenever a change was made in the dynamic_reconfigure window
*/
void LongitudinalController::reconfigureRequest(const Config& config, uint32_t level) {
    params_.fromConfig(config);
}


} // namespace anicar_longitudinal_control_ros_tool
