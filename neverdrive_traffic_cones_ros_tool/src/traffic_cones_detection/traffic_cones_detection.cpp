#include "traffic_cones_detection.hpp"
using namespace std;
//using namespace cv;
namespace neverdrive_traffic_cones_ros_tool {

TrafficConesDetection::TrafficConesDetection(ros::NodeHandle nhPublic, ros::NodeHandle nhPrivate)
        : interface_{nhPrivate}, reconfigureServer_{nhPrivate}, current_task_{0} {

    /**
     * Initialization
     */
    interface_.fromParamServer();

    /**
     * Set up callbacks for subscribers and reconfigure.
     *
     * New subscribers can be created with "add_subscriber" in "cfg/TrafficConesDetection.if file.
     * Don't forget to register your callbacks here!
     */

    setupIntrinsics(interface_.image_scaling);
    this->setupPublisher(nhPrivate);
    this->setupSubscriber(nhPrivate);
    current_task_subscriber_ = nhPrivate.subscribe("/current_task",
                                                   1,
                                                   &TrafficConesDetection::setTaskCallback,
                                                   this);


    reconfigureServer_.setCallback(boost::bind(&TrafficConesDetection::reconfigureRequest, this, _1, _2));

    rosinterface_handler::showNodeInfo();
}

void TrafficConesDetection::setTaskCallback(std_msgs::Int8 msg){
    current_task_=msg.data;
}

void TrafficConesDetection::setupIntrinsics(double resize_scale){
    resized_focal_length_ = interface_.focal_length * resize_scale;
    resized_u0_ = interface_.u0 * resize_scale;
    resized_v0_ = interface_.v0 * resize_scale;

    structuring_element_0_ = cv::getStructuringElement(0, cv::Size(interface_.opening_element_size, interface_.opening_element_size), cv::Point(0, 0)); //0 is MORPH_RECT
}

void TrafficConesDetection::setupPublisher(ros::NodeHandle& nh){
    publisher_original_depth_ = nh.advertise<Img>("/original_depth", interface_.queue_size);
    publisher_filtered_image_ = nh.advertise<Img>("/inRange_image", interface_.queue_size);
    publisher_morph_open_image_ = nh.advertise<Img>("/ir_frame_opening_image", interface_.queue_size);
    publisher_point_cloud_ = nh.advertise<sensor_msgs::PointCloud2>("/traffic_cones_point_cloud", interface_.queue_size);
}

void TrafficConesDetection::setupSubscriber(ros::NodeHandle& nh){
    subscriber_image_ = std::make_unique<message_filters::Subscriber<Img>>(
            nh, interface_.input_image, interface_.queue_size);
    subscriber_depth_ = std::make_unique<message_filters::Subscriber<Img>>(
            nh, interface_.input_depth, interface_.queue_size);
    kinect_image_synchronizer_ = std::make_unique<message_filters::Synchronizer<SynchronizedKinectImages>>(
            SynchronizedKinectImages(interface_.queue_size), *subscriber_image_, *subscriber_depth_);

    kinect_image_synchronizer_->registerCallback(boost::bind(&TrafficConesDetection::callbackKinectImages, this, _1, _2));


}

void TrafficConesDetection::callbackKinectImages(const Img::ConstPtr& image_msg, const Img::ConstPtr& depth_msg){

    if(current_task_!=2 && current_task_!=3){ return;}

    setupIntrinsics(interface_.image_scaling);
    getIr2World();
    cv::Mat image = cv_bridge::toCvShare(image_msg)->image;
    cv::Mat depth = cv_bridge::toCvShare(depth_msg)->image;


    cv_bridge::CvImage cvOriginalDepth;
    cvOriginalDepth.image = depth;
    Img::Ptr depth_ptr(cvOriginalDepth.toImageMsg());
    depth_ptr->header = image_msg->header;
    depth_ptr->encoding = "mono8";

    publisher_original_depth_.publish(depth_msg);




    cv::Mat depth_filtered = depth;
    cv::Mat image_filtered;
    this->filterImage(image, image_filtered);          //depth_filtered no value



    cv_bridge::CvImage cvFilteredImage;
    cvFilteredImage.image = image_filtered;
    Img::Ptr image_filtered_ptr(cvFilteredImage.toImageMsg());
    image_filtered_ptr->header = image_msg->header;
    image_filtered_ptr->encoding = "mono8";

    publisher_filtered_image_.publish(image_filtered_ptr);


    //resize
    cv::resize(image_filtered, image_filtered, cv::Size(0, 0), interface_.image_scaling, interface_.image_scaling, cv::INTER_NEAREST);
    cv::resize(depth_filtered, depth_filtered, cv::Size(0, 0), interface_.image_scaling, interface_.image_scaling, cv::INTER_NEAREST);

    cv::Mat image_filtered_ir_frame;
    //kinect_offset:0.06 translation between RGB camera and IR sensor
    //this->shiftImage(image_filtered, depth_filtered, image_filtered_ir_frame, interface_.kinect_offset);

    //cv::morphologyEx(image_filtered_ir_frame, image_filtered_ir_frame, cv::MORPH_OPEN, structuring_element_0_);

/*
    cv_bridge::CvImage cvMorphOpenImage;
    cvMorphOpenImage.image = image_filtered_ir_frame;
    Img::Ptr image_morph_ptr(cvMorphOpenImage.toImageMsg());
    image_morph_ptr->header = image_msg->header;
    image_morph_ptr->encoding = "mono8";
    publisher_morph_open_image_.publish(image_morph_ptr);


    const pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud{new pcl::PointCloud<pcl::PointXYZI>};
    this->project2PointCloud(image_filtered_ir_frame, depth_filtered, ir_2_world_, point_cloud);

    sensor_msgs::PointCloud2 point_cloud_msg;
    pcl::toROSMsg(*point_cloud, point_cloud_msg);
    point_cloud_msg.header.frame_id = interface_.world_frame;

    publisher_point_cloud_.publish(point_cloud_msg);
*/

    const pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud{new pcl::PointCloud<pcl::PointXYZI>};

    for (uint16_t u = 0; u < depth_filtered.cols; u++) {
        for (uint16_t v = 0; v < depth_filtered.rows; v++) {
            if(depth_filtered.at<uint16_t>(v, u) > 3000){continue;}

            pcl::PointXYZI newPoint;
            double z = (double)depth_filtered.at<uint16_t>(v, u) / 1000;
            if (z < interface_.depth_max &&  z > interface_.depth_min) {
                int16_t u2 = u - interface_.kinect_offset *interface_.image_scaling* resized_focal_length_ / z;                       //see Fahrzeugensehen chapter 2 page 22
                if (u2 >= 0 && u2 < depth_filtered.cols) {
                    if (image_filtered.at<uint8_t>(v, u2) != 0) {
                        newPoint.x = z / resized_focal_length_ * (u - resized_u0_);
                        newPoint.y = z / resized_focal_length_ * (v - resized_v0_);
                        newPoint.z = z;                         //because already did cv::inRange() Binarization: shift the Binarization images
                        newPoint.intensity = 0;
                    }else{
                        //if(z > 2){ continue;}
                        newPoint.x = z / resized_focal_length_ * (u - resized_u0_);
                        newPoint.y = z / resized_focal_length_ * (v - resized_v0_);
                        newPoint.z = z;                         //because already did cv::inRange() Binarization: shift the Binarization images
                        newPoint.intensity = 1;
                    }

                    pcl::PointXYZI transPoint = pcl::transformPoint(newPoint, ir_2_world_.cast<float>());

                    if( (transPoint.z > interface_.cone_height_min && transPoint.z < interface_.cone_height_max && newPoint.intensity == 0)
                        || (transPoint.z < interface_.cone_height_min &&  newPoint.intensity == 1) ){
                        point_cloud->push_back(transPoint);
                    }
                }
            }
        }
    }


    sensor_msgs::PointCloud2 point_cloud_msg;
    pcl::toROSMsg(*point_cloud, point_cloud_msg);
    point_cloud_msg.header.frame_id = interface_.world_frame;
    publisher_point_cloud_.publish(point_cloud_msg);

}

void  TrafficConesDetection::getIr2World(){
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

    ROS_INFO_STREAM("IR 2 WORLD"<< ir_2_world_.translation()<<endl);
}

void TrafficConesDetection::filterImage(cv::Mat& image, cv::Mat& image_filtered){
    cv::cvtColor(image, image_filtered, cv::COLOR_BGR2HSV);

    cv::inRange(image_filtered,                                              //inRange(): Binarization. in Range 255 out Range 0
                cv::Scalar(interface_.hue_min, interface_.saturation_min, interface_.value_min),
                cv::Scalar(interface_.hue_max, interface_.saturation_max, interface_.value_max),
                image_filtered);
}

void TrafficConesDetection::shiftImage(const cv::Mat& src, const cv::Mat& depth, cv::Mat& dst, const double baseline){
    dst = cv::Mat::zeros(src.size(), src.type());

    for (uint16_t u = 0; u < depth.cols; u++) {
        for (uint16_t v = 0; v < depth.rows; v++) {
            double z = (double)depth.at<uint16_t>(v, u) / 1000;
            if (z < interface_.depth_max &&  z > interface_.depth_min) {
                int16_t u2 = u - baseline * resized_focal_length_ / z;                       //see Fahrzeugensehen chapter 2 page 22
                if (u2 >= 0 && u2 < depth.cols) {
                    if (src.at<uint8_t>(v, u2) != 0) {
                        dst.at<uint8_t>(v, u) = 255;                           //because already did cv::inRange() Binarization: shift the Binarization images
                    }
                }
            }
        }
    }
}

void TrafficConesDetection::project2PointCloud(const cv::Mat& image, const cv::Mat& depth, const Eigen::Affine3d& transform, const pcl::PointCloud<pcl::PointXYZI>::Ptr& points){
    for (uint16_t u = 0; u < image.cols; u++) {
        for (uint16_t v = 0; v < image.rows; v++) {
            if (image.at<uint8_t>(v, u) != 0) {
                double z = (double)depth.at<uint16_t>(v, u) / 1000;
                pcl::PointXYZI newPoint;
                newPoint.x = z / resized_focal_length_ * (u - resized_u0_);
                newPoint.y = z / resized_focal_length_ * (v - resized_v0_);
                newPoint.z = z;

                pcl::PointXYZI transPoint = pcl::transformPoint(newPoint, transform.cast<float>());

                if(transPoint.z > interface_.cone_height_min && transPoint.z < interface_.cone_height_max){
                    points->push_back(transPoint);
                }
            }
        }
    }

   // points->header.frame_id = interface_.world_frame;




}

    /**
  * This callback is called at startup or whenever a change was made in the dynamic_reconfigure window
*/
void TrafficConesDetection::reconfigureRequest(const Interface::Config& config, uint32_t level) {
    interface_.fromConfig(config);
}


} // namespace neverdrive_traffic_cones_ros_tool
