#include "lateral_controller.hpp"
#include <tf2_eigen/tf2_eigen.h>
#include <utils_ros/ros_console.hpp>
#include <boost/algorithm/clamp.hpp>
#include <boost/math/special_functions/sign.hpp>
#include <boost/range/algorithm/min_element.hpp>

#include "lateral_control_ros_tool/discrete_curvature.h"
#include "motor_interface_ros_tool/ServoCommand.h"
#include "safe_iterator_operations.h"

#include <algorithm>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <cmath>
#define PI  3.1415926535898
#define MAX_INDEX 20  //how many point need to be compared
#define COMPARE_DIST 5//params_.compare_dist;//how many points are there between the points should be measured
#define JUDGMENT_INDEX_NUMBER 20
#define JUDGMENT_DIST 5
#define MIN_ANGLE 2
#define II_OFF 1
#define INDEX_SHIFT 0

/*
#include <fstream>
#include <cstdlib>
#include <iostream>
#include <string>
*/

namespace anicar_lateral_control_ros_tool {

using ServoCommand = motor_interface_ros_tool::ServoCommand;
using MotorCommand = motor_interface_ros_tool::MotorCommand;

LateralController::LateralController(ros::NodeHandle nh_public, ros::NodeHandle nh_private)
        : params_{nh_private}, reconfigureServer_{nh_private}, tfListener_{tfBuffer_} {

    /**
     * Initialization
     */
    utils_ros::setLoggerLevel(nh_private);
    params_.fromParamServer();

    /**
     * Publishers & subscriber
     */

    servo_command_publisher_ = nh_private.advertise<ServoCommand>(params_.servo_command_topic, params_.msg_queue_size);
    zebra_cross_publisher_ = nh_private.advertise<std_msgs::String>("zebra_cross",params_.msg_queue_size);

    // Instantiate subscriber last, to assure all objects are initialised when first message is received.
    path_subscriber_ = nh_private.subscribe(params_.path_topic,
                                            params_.msg_queue_size,
                                            &LateralController::pathCallback,
                                            this,
                                            ros::TransportHints().tcpNoDelay());
    velocity_subscriber_=nh_private.subscribe("/motor_interface/motor_command",
    		                                  params_.msg_queue_size,
											  &LateralController::velocityCallback,
											  this,
											  ros::TransportHints().tcpNoDelay());
    unknow_envrionmet_subscriber=nh_private.subscribe("/current_task",
        		                                  params_.msg_queue_size,
    											  &LateralController::unknowenvironmentCallback,
    											  this,
    											  ros::TransportHints().tcpNoDelay());
    control_loop_timer_ =
        nh_private.createTimer(ros::Rate(params_.control_loop_rate), &LateralController::controlLoopCallback, this);

    /**
     * Set up dynamic reconfiguration
     */
    reconfigureServer_.setCallback(boost::bind(&LateralController::reconfigureRequest, this, _1, _2));

    utils_ros::showNodeInfo();
}

void LateralController::unknowenvironmentCallback(const std_msgs::Int8ConstPtr &msg){
	task_param = msg->data;
}

void LateralController::velocityCallback(const MotorCommand::ConstPtr& msg){
	curr_velo=msg->velocity;
	sign_flag=msg->sign_flag;
}

void LateralController::pathCallback(const nav_msgs::Path::ConstPtr& msg) {
/*
    using namespace std;

	std::ofstream _output;
	static int count=0;

	_output.open("output.txt"+(char)(count)); count++;
*/
    path_.clear();
    path_.reserve(msg->poses.size());
    //if wanna use judgment, must firstly init the judgmet at construction function
//    delete [] judgment;
//    judgment=new bool[msg->poses.size()];
    for (const auto& pose_stamped : msg->poses) {
        Eigen::Affine3d pose;
        tf2::fromMsg(pose_stamped.pose, pose);
        path_.push_back(pose);
//        _output << pose.translation() << '\n';
    }




}

double signedAngleBetween(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    const double vz = boost::math::sign(a.cross(b).z());
    return vz * std::acos(a.normalized().dot(b.normalized()));
}

void LateralController::controlLoopCallback(const ros::TimerEvent& timer_event) {
    if (path_.size() < 5) {
        ROS_INFO_STREAM("No Path received yet");
        return;
    }

    /*
     * Lookup the latest transform from vehicle to map frame
     */
    Eigen::Affine3d vehicle_pose;
    try {
        const geometry_msgs::TransformStamped tf_ros =
            tfBuffer_.lookupTransform(params_.map_frame_id, params_.vehicle_frame_id, ros::Time(0));
        vehicle_pose = tf2::transformToEigen(tf_ros);
    } catch (const tf2::TransformException& e) {
        ROS_WARN_STREAM(e.what());
        return;
    }

    const Eigen::Vector3d vehicle_position = vehicle_pose.translation();

    /*
     * send current position topic
     */
    ROS_WARN_STREAM(vehicle_position.x()<<' '<<vehicle_position.y());
    if ((vehicle_position.x()>params_.zebra_x1)&&(vehicle_position.x()<params_.zebra_x2)
    		&&(vehicle_position.y()>params_.zebra_y1)&&(vehicle_position.y())<params_.zebra_y2)
    	zebra_cross_msg.data="WARNING";
    else
    	zebra_cross_msg.data="GO";
    zebra_cross_publisher_.publish(zebra_cross_msg);

    /*
     * Shift Rear-axle in current direction -> kos_shift
     */
    const Eigen::Vector3d vehicle_frame_unit_x = [&vehicle_pose]() {
        Eigen::Vector3d p = vehicle_pose.rotation() * Eigen::Vector3d::UnitX();
        p.z() = 0.0;
        return p.normalized();
    }();

 //   const Eigen::Vector3d shifted_vehicle_position = vehicle_position + vehicle_frame_unit_x * params_.kos_shift;

    //use the front point
    const Eigen::Vector3d shifted_vehicle_position = vehicle_position + vehicle_frame_unit_x * (params_.wheel_base);

    auto const& it = boost::range::min_element(
        path_, [&shifted_vehicle_position](const Eigen::Affine3d& lhs, const Eigen::Affine3d& rhs) {
            return (lhs.translation() - shifted_vehicle_position).squaredNorm() <
                   (rhs.translation() - shifted_vehicle_position).squaredNorm();
        });

// my codes are here   ------ velocity controller






      long Index=std::distance(path_.begin(),it);//Index_Ptr);
      int Compare_dist=params_.compare_dist;//COMPARE_DIST;
      short Max_Num=params_.max_index;//MAX_INDEX;
      long Compare_Index[Max_Num];
//      Eigen::Affine3d Compare[Max_Num];
      Eigen::Vector3d Compare[Max_Num];
      ServoCommand servo_command;
      boost::accumulators::accumulator_set< double, boost::accumulators::features<boost::accumulators::tag::mean, boost::accumulators::tag::median> > Compare_Angle;

      if (task_param < 2){
  // does the third compare_point reach the end of published path?
  /*
      	  for (Max_Num=1;Max_Num<=MAX_INDEX;Max_Num++)
        	if (Index+Compare_dist*Max_Num>path_.size())
      	  	  break;
  */


      //	record next Max_Num*Compare_dist points-----------------my way
/*
      	  for (std::size_t i=0;i<Max_Num;i++)
      		Compare_Index[i]=Index+i*Compare_dist;
      	  for (std::size_t i=0;i<Max_Num;i++)
      		Compare[i]=path_[Compare_Index[i]];
*/
      //	record next Max_Num*Compare_dist points-----------------safe function way

    	  for (std::size_t i=0;i<Max_Num;i++)
    		  Compare[i] = safe_next(it, i*Compare_dist, std::prev(path_.end()))->translation();



// make the road straight!
     /*
     	 if (judgment[Index]==false){//when this point is not on a checked straight path or has not been checked.
    	  	  Eigen::Vector3d Judgment_Vector[(Max_Num-1)*Compare_dist];
    	  	  Eigen::Vector3d Begin_End=path_[Index+(Max_Num-1)*Compare_dist].translation()-path_[Index].translation();
    	  	  Eigen::Vector3d average_vector;

    	  	  judgment[Index]=true;
    	  	  for (std::size_t i=1;i<=(Max_Num-1)*Compare_dist;i++){
    		  	  Judgment_Vector[i]=path_[Index+i].translation()-path_[Index].translation();
    		  	  average_vector=average_vector.normalized()+Judgment_Vector[i].normalized();
    	  	  }

    	  	  double Straight_Angle=abs(signedAngleBetween(average_vector,Begin_End))/PI*180;
    	  	  double min_angle=MIN_ANGLE;

    	  	  ROS_WARN_STREAM("the Vector Average Angle="<<Straight_Angle);
    	  	  if (Straight_Angle<min_angle){
    		  	  servo_command.kind_of_path="Straight";
    		  	  double step_x=Begin_End.x()/((Max_Num-1)*Compare_dist);      //(path_[Index+(Max_Num-1)*Compare_dist].translation().x()-path_[Index].translation().x())/((Max_Num-1)*Compare_dist);
    		  	  double step_y=Begin_End.y()/((Max_Num-1)*Compare_dist);

					  //change original path to a total straight path

    		  	  for (size_t i=1;i<=(Max_Num-1)*Compare_dist;i++){
    			  	  path_[Index+i].translation().x()=path_[Index].translation().x()+step_x*i;
    			  	  path_[Index+i].translation().y()=path_[Index].translation().y()+step_y*i;
    			  	  judgment[Index+i]=true;
    		  	  }
    	  	  }
    	  	  else{
    		  	  servo_command.kind_of_path="Curve";
    		 	 // judgment[Index]=true;
    	  	  }
      	  }
*/



    	  for (std::size_t i=0;i<Max_Num-2;i++)
    //		  Compare_Angle(signedAngleBetween(Compare[i+1].translation()-Compare[i].translation(),
    	//			  	  	  	  	  	       Compare[i+2].translation()-Compare[i+1].translation()));
    		  Compare_Angle(signedAngleBetween(Compare[i+1]-Compare[i],
    		      	  			  	  	  	   Compare[i+2]-Compare[i+1]));
    	//  servo_command.speed_para=((PI-abs(boost::accumulators::median(Compare_Angle)))/PI-0.9)*10;
     // ROS_WARN_STREAM("the velocity parameter="<<servo_command.speed_para);
      }


/*
// not change lateral angle during stop period

    if (sign_flag=="STOP")
	   return;
*/

   // stanley controller


    if (it == std::prev(path_.end())) {
      ROS_ERROR("Reached end of trajectory!");
      //return;
    }

    const Eigen::Vector3d closest_trajectory_point = it->translation();
    /**
     * Find look ahead point
     */


    /*
     * change parameters based on current task
     */
    int index_shift_decide = params_.index_shift,ii_off_decide = params_.ii_off;
    if (task_param > 1){
    	index_shift_decide = INDEX_SHIFT;
    	ii_off_decide = II_OFF;
    }

    //	params_.index_shift --- lookaheadpoint
    //	params_.ii_off ---	point offset for curvature approximation
//    auto const& it_lb = safe_next(it, params_.index_shift, std::prev(path_.end()));
    auto const& it_lb = safe_next(it, index_shift_decide, std::prev(path_.end()));
    const Eigen::Vector3d& p_lookahead = it_lb->translation();

    // Determine three points for approximation
    /*
     * original code
     */

//    const Eigen::Vector3d& p_prev = safe_prev(it_lb, params_.ii_off, path_.begin())->translation();
//    const Eigen::Vector3d& p_next = safe_next(it_lb, params_.ii_off, std::prev(path_.end()))->translation();

    const Eigen::Vector3d& p_prev = safe_prev(it_lb, ii_off_decide, path_.begin())->translation();
    const Eigen::Vector3d& p_next = safe_next(it_lb, ii_off_decide, std::prev(path_.end()))->translation();

    /*
    Eigen::Vector3d p_prev;Eigen::Vector3d p_next;
    if (task_param!=2){
    	const Eigen::Vector3d& p_prev_tmp = safe_prev(it_lb, params_.ii_off, path_.begin())->translation();
    	const Eigen::Vector3d& p_next_tmp = safe_next(it_lb, params_.ii_off, std::prev(path_.end()))->translation();
    	p_prev = p_prev_tmp;
    	p_next = p_next_tmp;
    }else{
    	const Eigen::Vector3d& p_prev_tmp = safe_prev(it_lb, II_OFF, path_.begin())->translation();
    	const Eigen::Vector3d& p_next_tmp = safe_next(it_lb, II_OFF, std::prev(path_.end()))->translation();
    	p_prev = p_prev_tmp;
    	p_next = p_next_tmp;
    }
	*/
    //const double curv = discreteCurvature(p_prev.head<2>(), p_lookahead.head<2>(), p_next.head<2>());

    Eigen::Vector3d target_direction = p_next - p_lookahead;// tangent of the nearest point on the path
    if(task_param >1){
	target_direction =  p_lookahead - p_prev;
    }

    /*
     * Compute angle difference
     */
    const double delta_angle = signedAngleBetween(target_direction, vehicle_frame_unit_x);// delta fi of stanley controller paper
    /*
     * Calculation of the sign for distance control (= determine side of trajectory)
     */
    const double vz_dist =
        boost::math::sign(target_direction.cross(closest_trajectory_point - shifted_vehicle_position).z());//if target direction on the right side of vehicle direction, the sign must be positive
    const double dist = (closest_trajectory_point - shifted_vehicle_position).norm();// value e of stanley controller paper

    /*
     * Controller law
     */

/* original code
    double r_ang = -1. * params_.k_ang * delta_angle;
    double r_dist = params_.k_dist * vz_dist * dist;
    double steering_angle = std::atan(params_.wheel_base * (curv + r_ang + r_dist));


*/
    double steering_angle_stanley;
    double second_para=params_.k_error*vz_dist*dist/(params_.k_soft+curr_velo);
    double mean_compare_angle;

    if (task_param < 2)
    	mean_compare_angle = boost::accumulators::mean(Compare_Angle);
    else
    	mean_compare_angle = 0;

    steering_angle_stanley=-delta_angle + std::atan(second_para) + params_.angle_fix + params_.k_mean_steer*mean_compare_angle;
   // ROS_WARN_STREAM("Steering_angle: " << steering_angle);
    //double steering_angle=steering_angle_stanley;


// PID Controller for Stanley Controller's Output

    double err=0;
//    static double err_last=0, angle_last=0, err_integral=0;
    double steering_angle;

    for (std::size_t i=0;i<params_.pid_num;i++){
      err=steering_angle_stanley - angle_last;
      err_integral += err;
      steering_angle=params_.Kp*err + params_.Ki*err_integral + params_.Kd*(err - err_last);
      ROS_WARN_STREAM("Steering_angle: " << params_.Kd << '\t'<< params_.Ki << '\t'<< params_.Kp);
      steering_angle = boost::algorithm::clamp(steering_angle, -params_.max_steering_angle, params_.max_steering_angle);
//      ROS_DEBUG_STREAM("Steering_angle: " << steering_angle);
      
      if(task_param > 2 ){
	steering_angle = steering_angle_stanley;
      }	

      servo_command.header.stamp = timer_event.current_expected;
      servo_command.steering_angle = steering_angle;
      servo_command_publisher_.publish(servo_command);

      err_last=err;
      angle_last=steering_angle;
    }





    //steering_angle = boost::algorithm::clamp(steering_angle, -params_.max_steering_angle, params_.max_steering_angle);
    //ROS_WARN_STREAM("Steering_angle: " << steering_angle << steering_angle_stanley);
    

    
/*
  //  ServoCommand servo_command;
    servo_command.header.stamp = timer_event.current_expected;
    servo_command.steering_angle = steering_angle;

    servo_command_publisher_.publish(servo_command);
*/
}

/**
  * This callback is called at startup or whenever a change was made in the dynamic_reconfigure window
*/
void LateralController::reconfigureRequest(const Config& config, uint32_t level) {
    params_.fromConfig(config);
}


} // namespace anicar_lateral_control_ros_tool
