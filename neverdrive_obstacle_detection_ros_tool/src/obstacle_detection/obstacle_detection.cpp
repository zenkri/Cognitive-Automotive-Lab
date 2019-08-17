#include "obstacle_detection.hpp"

using namespace std;
using namespace cv;

namespace neverdrive_obstacle_detection_ros_tool {

ObstacleDetection::ObstacleDetection(ros::NodeHandle nhPublic, ros::NodeHandle nhPrivate)
: interface_{nhPrivate}, reconfigureServer_{nhPrivate} {

	/**
	 * Initialization
	 */
	interface_.fromParamServer();

	/**
	 * Set up callbacks for subscribers and reconfigure.
	 *
	 * New subscribers can be created with "add_subscriber" in "cfg/ObstacleDetection.if file.
	 * Don't forget to register your callbacks here!
	 */
	reconfigureServer_.setCallback(boost::bind(&ObstacleDetection::reconfigureRequest, this, _1, _2));

	ground_depth_ = imread(interface_.path_to_ground_depth + "ground_depth.png", IMREAD_ANYDEPTH);

	ROS_INFO_STREAM("ground depth size: "<<ground_depth_.rows<<" "<<ground_depth_.cols<<endl);
	current_task_subscriber_ = nhPrivate.subscribe("/current_task",1,&ObstacleDetection::taskCallback, this);
	
	depth_image_subscriber_ = nhPrivate.subscribe("/kinect2/qhd/image_depth_rect",
			1,
			&ObstacleDetection::depthImageCallback,
			this);
	color_image_subscriber_ = nhPrivate.subscribe("/kinect2/qhd/image_color_rect",
				1,
				&ObstacleDetection::colorImageCallback,
				this);

	//depth_image_publisher_ = nhPrivate.advertise<sensor_msgs::Image>("/obstacle_detection_depth_image",1);
	//color_image_publisher_ = nhPrivate.advertise<sensor_msgs::Image>("/obstacle_detection_color_image",1);

	obstacle_detected_publisher_=  nhPrivate.advertise<neverdrive_obstacle_detection_ros_tool::Box>("/obstacle_detection",1);
	//obstacle_detected_publisher_=  nhPrivate.advertise<std_msgs::String>("/obstacle_detection",1);

	depth_image_pub_timer_ = nhPrivate.createTimer(ros::Rate(interface_.timer_rate), &ObstacleDetection::callbackTimer, this);

	//depth_image_pub_timer_ =

	rosinterface_handler::showNodeInfo();
}

void ObstacleDetection::taskCallback(const std_msgs::Int8ConstPtr& msg){
	current_task_ = msg->data;
	ROS_INFO_STREAM("callback"<<current_task_);
}

void ObstacleDetection::depthImageCallback(const Msg::ConstPtr& msg) {

	if(current_task_ > 1){return;}
	cv_bridge::CvImageConstPtr cvPtrImage;
	cvPtrImage = cv_bridge::toCvShare(msg);
	Mat depthImage;
	depthImage = cvPtrImage->image;

	maskDepth(ground_depth_, depthImage, depthImage, 300, 3000); // only consider 0.3m --3.0m

	depthImage.convertTo(depthImage,CV_8UC1,1.0/16); // convert the image into 8 bit, distance/16
	vector<vector<Point> > contour;
	findObstacleContour(depthImage, 20, 255, 5000,interface_.opening_filter_size);  // 20*16 = 320mm

}

void ObstacleDetection::colorImageCallback(const Msg::ConstPtr& msg) {
        if(current_task_ > 1){return;}
	cv_bridge::CvImageConstPtr cvPtrImage;
	cvPtrImage = cv_bridge::toCvShare(msg);
	Mat colorImage,colorImg,blueAndRedImg,whiteOut,blueOut,redOut;
	colorImage = cvPtrImage->image;


	rectangle(colorImage, boxOfObstacle, Scalar(255,255,255), 1 ,8, 0 );
	//rectangle(colorImage, Rect(boxOfObstacle.x-0.3*boxOfObstacle.width,boxOfObstacle.y,boxOfObstacle.width,boxOfObstacle.height), Scalar(255,255,255), 1 ,8, 0 );

	if(boxOfObstacle.area()>0){
		//cvtColor(colorImage, colorImage, CV_BGR2HSV); // transform RGB to HSV
		if (boxOfObstacle.x-0.19*boxOfObstacle.width>0){
			colorImg = colorImage(Rect(boxOfObstacle.x-0.19*boxOfObstacle.width,boxOfObstacle.y,boxOfObstacle.width,boxOfObstacle.height));
		}else
		{
			colorImg = colorImage(boxOfObstacle);
		}
		//colorImg = colorImage(Rect(0,boxOfObstacle.y,colorImage.cols,boxOfObstacle.height));
		cvtColor(colorImg, colorImg, CV_BGR2HSV); // transform RGB to HSV
		//  binary ,remain the white part into white(255) .
		inRange(colorImg,Scalar(interface_.min_h,interface_.min_s,interface_.min_v),Scalar(interface_.max_h,interface_.max_s,interface_.max_v),whiteOut); 
		inRange(colorImg,Scalar(100,43,46),Scalar(124,255,255),blueOut);
		inRange(colorImg,Scalar(0,43,46),Scalar(10,255,255),redOut);	

		int sumPixelWhite = 0;
		int sumPixelBlue = 0;
		int sumPixelRed = 0;
		for (int i = 0;i<colorImg.rows;i++){
			for(int j = 0;j<colorImg.cols;j++){
				if((int)whiteOut.at<uchar>(i,j)!=0){
					sumPixelWhite += 1;
				}
				if((int)blueOut.at<uchar>(i,j)!=0){
					sumPixelBlue += 1;
				}
				if((int)redOut.at<uchar>(i,j)!=0){
					sumPixelRed += 1;
				}
			}
		}

		int areaDepth = boxOfObstacle.width * boxOfObstacle.height;
		double procent = (double)sumPixelWhite / areaDepth;
		double procentBlue = (double)sumPixelBlue / areaDepth;
		double procentRed = (double)sumPixelRed / areaDepth;
		if (procent > interface_.min_procent && procentBlue < 0.3 && procentRed < 0.3 ){
			isObstacles = true;
		}else{
			isObstacles = false;
		}
	}else{
		isObstacles = false;
	}
	
	// publish colorImage for test
	//sensor_msgs::ImagePtr colorMsg;
	//std_msgs::Header header;
	//header.stamp = ros::Time::now();
	//colorMsg = cv_bridge::CvImage(header,"mono8",out).toImageMsg();
	//colorMsg = cv_bridge::CvImage(header,"bgr8",colorImage).toImageMsg();
	//color_image_publisher_.publish(colorMsg);
}

pcl::PointXYZ ObstacleDetection::pointInCamera (int pixelU, int pixelV, double depth){
	double zd = depth;
	pcl::PointXYZ pointInCam;
	pointInCam.x = zd / interface_.ir_focal_length * (pixelU - interface_.ir_u0);
	pointInCam.y = zd / interface_.ir_focal_length * (pixelV - interface_.ir_v0);
	pointInCam.z = zd;

	return pointInCam;
}

bool ObstacleDetection::pixelInMapArea(Mat & depth,int v,int u){
	double z = (double)depth.at<ushort>(v, u) / 1000;

	pcl::PointXYZ point_in = pointInCamera(u,v,z);

	pcl::PointXYZ out = pcl::transformPoint(point_in, ir_2_world_.cast<float>());

	if(out.x < interface_.map_area_max_x && out.x > interface_.map_area_min_x
			&& out.y < interface_.map_area_max_y && out.y > interface_.map_area_min_y
			&& out.z < interface_.map_area_max_z && out.z > interface_.map_area_min_z){
		return true;
	}

	return false;
}

void ObstacleDetection::getIr2World(){
	while(1) {
		try {
			const geometry_msgs::TransformStamped tf_ros =
					tfBuffer_.lookupTransform(interface_.world_frame, interface_.kinect_ir_frame, ros::Time(0));
			ir_2_world_ = tf2::transformToEigen(tf_ros);
			break;
		} catch (const tf2::TransformException &e) {
			ROS_WARN_STREAM(e.what());
		}
	}

	//ROS_INFO_STREAM("IR 2 WORLD"<< ir_2_world_.translation()<<(int)current_task_<<8<<endl);
}

void ObstacleDetection::maskDepth(cv::Mat & ground, cv::Mat &image, cv::Mat &img, int minThreshold , int maxThreshold ){

	getIr2World();
	int nr = image.rows;
	int nc = image.cols;
	for(int i=0;i<nr;i++){
		for(int j=0;j<nc;j++){
			if( image.at<ushort>(i,j) > minThreshold &&  image.at<ushort>(i,j) < maxThreshold){
				// need improve !
				if(abs(image.at<ushort>(i,j) - ground.at<ushort>(i,j)) < ground_threshold_){
					img.at<ushort>(i,j)= 0;
				}
				else{
					if(pixelInMapArea(image,i,j)) {}
					else{
						img.at<ushort>(i, j) = 0;
					}
				}
			}
			else{
				img.at<ushort>(i,j)= 0;
			}
		}
	}

}

double ObstacleDetection::getObstacleDistance(Mat &depthImage, Rect contourRect){
	// get the median value of the region

	double minDepth = 0;
	vector <double> depthValueArray;
	for (int size_u = contourRect.x;size_u <= contourRect.x+contourRect.width;size_u++){
		for (int size_v = contourRect.y;size_v<= contourRect.y+contourRect.height;size_v++){

			if ((int)depthImage.at<uchar>(size_v,size_u) > 0){
				depthValueArray.push_back((int)depthImage.at<uchar>(size_v,size_u));
			}
		}
	}

	const int l = depthValueArray.size();
	sort(depthValueArray.begin(),depthValueArray.end());
	int idx = l/2;
	minDepth = depthValueArray[idx]* 16.0 / 1000;
	return minDepth ;

}

bool ObstacleDetection::isObstacleAhead(Rect rect, double depth){
	Point pLeftBot, pRightTop;
	pcl::PointXYZ pLb, pRt;

	pLeftBot.x = rect.x;
	pLeftBot.y = rect.y + rect.height;

	pRightTop.x = rect.x + rect.width;
	pRightTop.y = rect.y;

	pLb = pointInCamera(pLeftBot.x,pLeftBot.y,depth);
	pRt = pointInCamera(pRightTop.x ,pRightTop.y,depth);

	pcl::PointXYZ box_location = pcl::transformPoint(pLb, ir_2_world_.cast<float>());

	box_detected_msg.box_location_x = box_location.x;
	box_detected_msg.box_location_y = box_location.y;

	double width = abs(pLb.x - pRt.x);
	double length = abs(pLb.y - pRt.y);

	double size  = width * length * 100;

	if (depth <interface_.min_detect_distance && size > interface_.min_detect_area){
		return  true;
	}else{
		return false;
	}

}



void  ObstacleDetection::findObstacleContour(Mat &image, int minThre , int maxThre , int area ,int maskSize){
	Mat dep;
	image.copyTo(dep);
	/// Open operator to avoid noise
	Mat elementOpen = getStructuringElement(MORPH_RECT,Size(5,5)); // choose the right size
	Mat elementClose = getStructuringElement(MORPH_RECT,Size(maskSize,maskSize)); // choose the right size
	Mat out;
	morphologyEx(dep,out,MORPH_OPEN,elementOpen,Point(-1,-1),3); // opening
	morphologyEx(out,out,MORPH_CLOSE,elementClose,Point(-1,-1),interface_.opening_times); // opening
	Mat threshold_output;
	/// binary
	threshold(out, threshold_output, minThre, maxThre, CV_THRESH_BINARY);

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	/// find contours
	findContours(threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	vector<vector<Point> > contour; // maybe choose the largest contour
	vector<vector<Point> > largestContour;

	/// drawing the bounding box
	//Scalar color = Scalar::all(255);
	vector <double> minDepth;
	int index = 0;
	for (int i = 0; i< contours.size(); i++)  {
		if (contourArea(contours[i]) < area) // delete the hull small than 5000
			//isObstacles = false;
			continue;

		contour.push_back(contours[i]);
		Rect rect = boundingRect(contours[i]);
		minDepth.push_back( getObstacleDistance(dep,rect));
	}

	if (minDepth.size()!=0){
		int temp = minDepth.at(index);
		for (int x = 1;x < minDepth.size();x++){
			if (temp > minDepth.at(x)){
				temp = minDepth.at(x);
				index = x;
			}
		}


		Rect rectOnly = boundingRect(contours[index]);
		boxOfObstacle = rectOnly;
		changeThePath_ = isObstacleAhead(rectOnly, minDepth.at(index));
		String output = "minDistance:"+to_string(minDepth.at(index));
		rectangle(dep, rectOnly, 255, 1 ,8, 0 );
		putText(dep,output,Point(50,60),FONT_HERSHEY_SIMPLEX,1,255,2,8);

	}
// publish depthImage for test
/*
	sensor_msgs::ImagePtr depthMsg;
	std_msgs::Header header;
	header.stamp = ros::Time::now();
	depthMsg = cv_bridge::CvImage(header,"mono8",dep).toImageMsg();
	depth_image_publisher_.publish(depthMsg);
*/
}

void ObstacleDetection::callbackTimer(const ros::TimerEvent& timer_event){
        if(current_task_ >1 ){return;}
	//  return if calling a service right now
	if (in_reset_){
		timerecoder_=timer_event.current_expected.toSec();
		return;
	}

	//depth_image_publisher_.publish()
	// publish change path info
	if(changeThePath_ && isObstacles){
		//std_msgs::String msg1;
		//msg1.data = "change the path";
		box_detected_msg.box_avoiding = "box_exist";
		//obstacle_detected_publisher_.publish(box_detected_msg);

		count_ +=1;
		if (count_>2){
			obstacle_detected_publisher_.publish(box_detected_msg);
			count_ -=1;
		}else{}
	}else{
		//std_msgs::String msg2;
		//msg2.data = "keep the path";
		boxOfObstacle = Rect(0,0,0,0);
		if(count_ !=0){
			count_ -=1;}else{}
		box_detected_msg.box_avoiding = "no_box";
		obstacle_detected_publisher_.publish(box_detected_msg);
	}


}

/**
 * This callback is called at startup or whenever a change was made in the dynamic_reconfigure window
 */
void ObstacleDetection::reconfigureRequest(const Interface::Config& config, uint32_t level) {
	interface_.fromConfig(config);
}


} // namespace neverdrive_obstacle_detection_ros_tool
