#include "pedestrian_detection.hpp"

#include <sstream>

using namespace cv;

namespace pedestrian_detection_ros_tool {

PedestrianDetection::PedestrianDetection(ros::NodeHandle nhPublic, ros::NodeHandle nhPrivate)
        : interface_{nhPrivate}, reconfigureServer_{nhPrivate}, robot_track(2,1,0), measurement(1,1){

    /**
     * Initialization
     */
    interface_.fromParamServer();

    /**
     * Set up callbacks for subscribers and reconfigure.
     *
     * New subscribers can be created with "add_subscriber" in "cfg/PedestrianDetection.if file.
     * Don't forget to register your callbacks here!
     */

    robot_track.transitionMatrix = (cv::Mat_<float>(2,2) << 1,1,0,1);
    cv::setIdentity(robot_track.measurementMatrix);
 //   robot_track.measurementMatrix = (cv::Mat_<float>(2,2) << 1,0,0,0);
    cv::setIdentity(robot_track.processNoiseCov, cv::Scalar::all(interface_.pro_noi));
    cv::setIdentity(robot_track.measurementNoiseCov, cv::Scalar::all(interface_.mea_noi));
    cv::setIdentity(robot_track.errorCovPost, cv::Scalar::all(1));

//    ROS_INFO_STREAM(robot_track.transitionMatrix << robot_track.measurementMatrix << robot_track.processNoiseCov << robot_track.measurementNoiseCov << robot_track.errorCovPost);

    depth_image_subscriber=nhPrivate.subscribe("/kinect2/qhd/image_depth_rect",1,&PedestrianDetection::depthimageSubscriberCallback,this);
    color_image_subscriber=nhPrivate.subscribe("/kinect2/qhd/image_color_rect",1,&PedestrianDetection::colorimageSubscriberCallback,this);
    zebra_cross_subscriber=nhPrivate.subscribe("/lateral_controller_node/zebra_cross",50,&PedestrianDetection::zebracrossCallback,this);
    label_color_image_publisher=nhPrivate.advertise<sensor_msgs::Image>("ccl_label_color_picture",1);
    label_depth_image_publisher=nhPrivate.advertise<sensor_msgs::Image>("ccl_label_depth_picture",1);
    pedestrian_detector_publisher=nhPrivate.advertise<std_msgs::String>("/pedestrian_detector",50);
    roi_classifier = nhPrivate.serviceClient<Classifier>("pedestrian_classifier",true);
    publish_loop_timer=nhPrivate.createTimer(ros::Duration(0.02), &PedestrianDetection::publishTimerCallback, this);

    reconfigureServer_.setCallback(boost::bind(&PedestrianDetection::reconfigureRequest, this, _1, _2));


    rosinterface_handler::showNodeInfo();
}

void PedestrianDetection::zebracrossCallback(const std_msgs::StringConstPtr &msg){
	command_zebra_cross=msg->data;
}

void PedestrianDetection::colorimageSubscriberCallback(const sensor_msgs::ImageConstPtr &msg){
	cv_bridge::CvImagePtr cv_ptr;
/*
	if (command_zebra_cross!="WARNING")
		return;
*/
	try{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}catch (cv_bridge::Exception &e){
		ROS_ERROR("cv_bridge exceptions: %s", e.what());
		return;
	}
	color_image=cv_ptr->image;
}

void PedestrianDetection::depthimageSubscriberCallback(const sensor_msgs::ImageConstPtr &msg){
	cv_bridge::CvImagePtr cv_ptr;

/*
	if (command_zebra_cross!="WARNING")
		return;
*/
	try{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
	}catch (cv_bridge::Exception &e){
		ROS_ERROR("cv_bridge exceptions: %s", e.what());
		return;
	}
	depth_image=cv_ptr->image;
/*
	if (!msg_image.empty()){

		try{
			transformStamped = tfBuffer_.lookupTransform("world", "kinect2_ir_optical_frame",ros::Time(0));
		}catch (tf2::TransformException &e){
			ROS_WARN("%s",e.what());
			return;
		}
		camera_to_world=tf2::transformToEigen(transformStamped);
	}
	ROS_INFO_STREAM(camera_to_world.linear());
*/
}
/*
Eigen::Vector3d PedestrianDetection::worldXYZ(const cv::Mat image, const double v, const double u){
	Eigen::Vector3d world_;
	geometry_msgs::TransformStamped transformStamped;

	//record transportation from camera to world
	try{
		transformStamped = tfBuffer_.lookupTransform("world", "kinect2_ir_optical_frame",ros::Time(0));
	}catch (tf2::TransformException &e){
		ROS_WARN("%s",e.what());
		return world_;
	}
	camera_to_world=tf2::transformToEigen(transformStamped);

	ROS_INFO_STREAM(transformStamped.transform.translation);


	world_(2) = image.at<short>(v,u) / camera_info.camera_factor;
	world_(1) = (v-camera_info.cy) * world_(2) / camera_info.fy;
	world_(0) = (u-camera_info.cx) * world_(2) / camera_info.fx;

	world_ = camera_to_world.translation() +  world_;
//	ROS_INFO_STREAM(camera_to_world.linear());

	return world_;
}
*/
void PedestrianDetection::publishTimerCallback(const ros::TimerEvent &timer_event){
	/*
	if (command_zebra_cross!="WARNING")
		return;
	*/
	if (!depth_image.empty()){


		double frame_x,frame_y,frame_z;
		cv::Mat remove_floor(depth_image.rows,depth_image.cols,CV_16U,cv::Scalar::all(0));
		geometry_msgs::TransformStamped transformStamped;
		Eigen::Affine3d transform_frame;

		//record transportation from camera to world
		try{
			transformStamped = tfBuffer_.lookupTransform("world", "kinect2_ir_optical_frame",ros::Time(0),ros::Duration(0.02));
			transform_frame = tf2::transformToEigen(transformStamped);
		}catch (tf2::TransformException &e){
			ROS_WARN("%s",e.what());
			return;
		}

	//	ROS_INFO_STREAM(transformStamped.transform);

		for (std::size_t v=200;v<depth_image.rows;v++)
			for (std::size_t u=0;u<depth_image.cols;u++){

				//xyz in camera frame       Action! z is depth, is x direction in the world frame, u is Height

				frame_z = depth_image.at<short>(v,u) / camera_info.camera_factor;
				frame_x = (u-camera_info.cx) * frame_z / camera_info.fx;
				frame_y = (v-camera_info.cy) * frame_z / camera_info.fy;
/*
				Eigen::Vector3d image_xyz(frame_x,frame_y,frame_z),world_xyz(0,0,0);
*/

				//point cloud algorithm
				pcl::PointXYZ image_xyz,world_xyz;

				image_xyz.z = frame_z; image_xyz.y = frame_y; image_xyz.x = frame_x;
				world_xyz = pcl::transformPoint(image_xyz, transform_frame.cast<float>());

				//xyz in world frame
				//tf2::doTransform(image_xyz, world_xyz, transformStamped);

				//if (PedestrianDetection::worldXYZ(msg_image,v,u).z() < interface_.ground_distance)
				if ((world_xyz.z < interface_.ground_distance)||(v<200)||(frame_z > interface_.too_far) ||
						(world_xyz.y < interface_.range_y_min) || (world_xyz.y > interface_.range_y_max) ||
						(world_xyz.x < interface_.range_x_min) || (world_xyz.x > interface_.range_x_max))
					//test.row(u).col(v) = 0;
					//test(u)(v) = 0;
					remove_floor.at<short>(v, u) = 0;
				else
					//test.row(u).col(v) = msg_image.at<short>(u,v);
					//test(u)(v) = msg_image.at<short>(u,v);
					remove_floor.at<short>(v, u) = depth_image.at<short>(v, u);
			}
/*
		cv::Mat remove_floor_8bit;
		remove_floor.convertTo(remove_floor_8bit, CV_8U);

		cv::Mat canny_image(remove_floor_8bit);
		cv::GaussianBlur(canny_image, canny_image, cv::Size(5,5), 2);
		cv::Canny(canny_image, canny_image, 80, 150);

		cv::Mat_<cv::Vec3s> image_3d;
		for (std::size_t i=0;i<3;i++){
			image_3d.push_back(remove_floor_8bit);
		}
*/
/*
		cv::Mat watershed_label_image(remove_floor);

		remove_floor.convertTo(remove_floor, CV_8U);
		watershed_label_image.convertTo(watershed_label_image, CV_32S);
		cv::watershed(remove_floor,watershed_label_image);
*/





		// opening and closing processing
		cv::Mat open_,close_;

		cv::morphologyEx(remove_floor, open_, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT,cv::Size(interface_.mask_size,interface_.mask_size)), cv::Point(-1,-1),interface_.open_num);
		cv::morphologyEx(open_, close_, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT,cv::Size(interface_.mask_size,interface_.mask_size)), cv::Point(-1,-1),interface_.close_num);

		//do CCL
		cv::Mat ccl_label,ccl_status,ccl_center;
		cv::Mat label_depth_image;
		cv::Mat label_color_image=color_image.clone();

/*
		std::vector<cv::Mat> matChannels;
		cv::Mat_<cv::Vec4b> image_4d;

		cv::split(color_image,matChannels);
		for (std::size_t i=0;i<matChannels.size();i++){
			image_4d.push_back(matChannels.at(i));
		}
		image_4d.push_back(depth_image);

		cv::Mat segment_image;

*/

//		close_=open_;
		close_.convertTo(close_, CV_8U);
//		remove_floor.convertTo(remove_floor, CV_8U);
//		int label_num_1 = cv::connectedComponents(close_, label_depth_image, 8, CV_16U);
//		int label_num = cv::connectedComponents(remove_floor, ccl_label, 4, CV_16U);

		int label_num_2 = cv::connectedComponentsWithStats(close_, ccl_label, ccl_status, ccl_center, 8);

		bool found_robot = false;
		int top_,left_depth,width_,height_,area_,left_;

		for (std::size_t i=1;i<label_num_2;i++){
			top_=ccl_status.at<int>(i,cv::CC_STAT_TOP);
			left_depth=ccl_status.at<int>(i,cv::CC_STAT_LEFT);
			width_=ccl_status.at<int>(i,cv::CC_STAT_WIDTH);
			height_=ccl_status.at<int>(i,cv::CC_STAT_HEIGHT);
			area_=ccl_status.at<int>(i,cv::CC_STAT_AREA);

//			ROS_INFO_STREAM(top_<<' '<<left_<< ' '<< width_<< ' ' << height_ << ' '<< area_);

//			cv::rectangle(label_depth_image, cv::Rect(top_,left_,width_,height_),cv::Scalar(0,0,255));
			if ((area_ > interface_.recognize_area_min)&&(area_ < interface_.recognize_area_max)){
				// shift a little to left
				if (left_depth - width_/2 > 0)
					left_ = left_depth - width_/2;
				else
					left_ = 0;

				// resize bounding box
				int left_adjust,top_adjust,width_adjust,height_adjust;

				left_adjust = (0 > (left_ - width_/15)) ? 0 : (left_ - width_/15);
				width_adjust = (color_image.cols-1 < (int)(left_adjust+width_*1.3)) ? (color_image.cols-1-left_adjust) : (int)(width_*1.3);
				top_adjust = (0 > (top_ - height_/15)) ? 0 : (top_ - height_/15);
				height_adjust = (color_image.rows-1 < (int)(top_adjust+height_*1.2)) ? (color_image.rows-1-top_adjust) : (int)(height_*1.2);

				cv::Rect ranges(left_adjust,top_adjust,width_adjust,height_adjust);
//				cv::Rect ranges(left_,top_,width_,height_);

				cv::Mat roi=color_image(ranges).clone();
				sensor_msgs::ImagePtr roi_msg;
				Msg roi_header;
/*
//				output ROI
				if (!roi.empty()){
					std::string output_name;std::stringstream sstr;
					cv::Mat roi=color_image(ranges);

					sstr.str("");
					sstr << "~/data/pictures/1_";
					sstr << ++output_image_counter;
					sstr << ".jpg";
					output_name = sstr.str();
					cv::imwrite(output_name, roi);
					ROS_INFO_STREAM(output_name);
                }

				header.stamp=timer_event.current_expected;
				color_label_image_msg=cv_bridge::CvImage(header,sensor_msgs::image_encodings::BGR8,roi).toImageMsg();
				label_color_image_publisher.publish(color_label_image_msg);
*/

//---------------------------  here input the classifier -------------------------------------



				if (!roi.empty()){

//					roi_header.stamp = ros::Time::now();
//					roi_msg = cv_bridge::CvImage(roi_header,sensor_msgs::image_encodings::BGR8,roi).toImageMsg();
//					srv.request.image = *roi_msg;
//					roi_classifier.call(srv);

//					bool classifier_flag = false;
//					int8_t response;

//					classifier_flag = true;
//					if (classifier_flag){
//					ROS_WARN_STREAM(srv.response.answer);
//					if (srv.response.answer == 1){
//					if (1 == 1){
//					response = srv.response.answer;
//					if (response == 1){
					cv::HOGDescriptor hog_detector;
//					hog_detector.load("/home/kal4-2/neverdrive/src/pedestrian_detection_ros_tool/src/pedestrian_detection/HOG_SVM.xml");
					hog_detector.load(interface_.hog_path);

//					cv::cvtColor(roi,roi,cv::COLOR_BGR2GRAY);
					cv::resize(roi,roi,cv::Size(80,128),0,0,cv::INTER_CUBIC);

					std::vector<cv::Point> location;
					std::vector<double> weights;

					hog_detector.detect(roi,location,weights,interface_.confidence_value);

					if (location.size() > 0){
						float center_u=left_adjust + width_adjust/2,center_v=top_adjust + height_adjust/2;

						found_robot = true;
						cv::rectangle(label_color_image, ranges,cv::Scalar(0,255,0));
						cv::circle(label_color_image, cv::Point(center_u, center_v), 5, cv::Scalar(0,255,0),-1);


//  					tracking the robot with kalman filter
						if (first_kalman){
							first_kalman = false;
							robot_track.statePost = (cv::Mat_<float>(2,1) << center_u,0);
						}
/*
						cv::Mat prediction = robot_track.predict();
						cv::Mat_<float> measurement(1,1);	measurement.at<float>(0) = center_u;
						cv::Mat estimated = robot_track.correct(measurement);
						float estimated_u = estimated.at<float>(0);
						float estimated_delta_u = estimated.at<float>(1);
						float predicted_u = prediction.at<float>(0);


						cv::circle(label_color_image, cv::Point(estimated_u, center_v), 5, cv::Scalar(0,0,255),-1);
						cv::circle(label_color_image, cv::Point(predicted_u, center_v), 5, cv::Scalar(255,0,0),-1);

					//calculate the estimated point in world frame, use ccl_label for catching depth info, but use estimated x,y to calculate world_x and world_y
					//but the center of ccl might have no value, so should firstly search it;
						frame_z =0;
						bool found_flag = false;
						for (std::size_t v=top_;v<top_+height_;v++){
							for (std::size_t u=left_depth;u<left_depth+width_;u++)
								if (depth_image.at<short>(v,u)!=0){
									frame_z = depth_image.at<short>(v,u) / camera_info.camera_factor;
									found_flag = true;
									break;
								}
							if (found_flag)
								break;
						}
//						frame_z = depth_image.at<short>(ccl_center.at<double>(1),ccl_center.at<double>(0)) / camera_info.camera_factor;
						frame_x = (estimated_u-camera_info.cx) * frame_z / camera_info.fx;
						frame_y = (center_v-camera_info.cy) * frame_z / camera_info.fy;

						pcl::PointXYZ image_xyz,world_xyz;

						image_xyz.z = frame_z; image_xyz.y = frame_y; image_xyz.x = frame_x;
						world_xyz = pcl::transformPoint(image_xyz, transform_frame.cast<float>());

//						ROS_INFO_STREAM(world_xyz.x<< '\t' << world_xyz.y << '\t' << world_xyz.z << '\t' << frame_z);
						if ((world_xyz.y > interface_.zebra_y_min)&&(world_xyz.y < interface_.zebra_y_max))
							pedestrian_msg.data = "ROBOT";
						else
							pedestrian_msg.data = "GO";
						pedestrian_detector_publisher.publish(pedestrian_msg);
*/

						prediction = robot_track.predict();
						measurement.at<float>(0) = center_u;
						estimated = robot_track.correct(measurement);
						see_robot = ros::Time::now();
						estimated_v = center_v;

					}

					if (ros::Time::now().toSec() - see_robot.toSec() > 3)
						first_kalman = true;
				}

				if (found_robot)
					break;

			}
		}
		if (ros::Time::now().toSec() - see_robot.toSec() > 3)
			first_kalman = true;

		pcl::PointXYZ image_xyz(0,0,0),world_xyz(0,0,0);
		frame_z =0;frame_y=0;frame_x=0;

		if (!first_kalman){
			float estimated_u = estimated.at<float>(0);
			float predicted_u = prediction.at<float>(0);

			cv::circle(label_color_image, cv::Point(estimated_u, estimated_v), 5, cv::Scalar(0,0,255),-1);
			cv::circle(label_color_image, cv::Point(predicted_u, estimated_v), 5, cv::Scalar(255,0,0),-1);

			bool found_flag = false;
			double frame_z_sum = 0;
			for (std::size_t v=top_;v<top_+height_;v++){
				for (std::size_t u=left_depth;u<left_depth+width_;u++)
					if (depth_image.at<short>(v,u)!=0){
						frame_z_sum += depth_image.at<short>(v,u) / camera_info.camera_factor;
//						found_flag = true;
//						break;
					}
//				if (found_flag)
//					break;
			}

			frame_z = frame_z_sum / (height_ * width_);
//			frame_z = depth_image.at<short>(ccl_center.at<double>(1),ccl_center.at<double>(0)) / camera_info.camera_factor;
//			frame_x = (estimated_u-camera_info.cx) * frame_z / camera_info.fx;
			frame_x = (predicted_u-camera_info.cx) * frame_z / camera_info.fx;
			frame_y = (estimated_v-camera_info.cy) * frame_z / camera_info.fy;

			image_xyz.z = frame_z; image_xyz.y = frame_y; image_xyz.x = frame_x;
			world_xyz = pcl::transformPoint(image_xyz, transform_frame.cast<float>());

		}

		if ((world_xyz.y > interface_.zebra_y_min)&&(world_xyz.y < interface_.zebra_y_max))
			pedestrian_msg.data = "ROBOT";
		else
			pedestrian_msg.data = "GO";

		ROS_WARN_STREAM(pedestrian_msg.data);
		pedestrian_detector_publisher.publish(pedestrian_msg);

		header.stamp=timer_event.current_expected;
		depth_label_image_msg=cv_bridge::CvImage(header,sensor_msgs::image_encodings::TYPE_16UC1,remove_floor).toImageMsg();
		color_label_image_msg=cv_bridge::CvImage(header,sensor_msgs::image_encodings::BGR8,label_color_image).toImageMsg();
		label_depth_image_publisher.publish(depth_label_image_msg);
		label_color_image_publisher.publish(color_label_image_msg);

//		ROS_INFO_STREAM(label_num_1 << ' ' << label_num_2);
	}
}

/**
  * This callback is called at startup or whenever a change was made in the dynamic_reconfigure window
*/
void PedestrianDetection::reconfigureRequest(const Interface::Config& config, uint32_t level) {
    interface_.fromConfig(config);
}


} // namespace pedestrian_detection_ros_tool
