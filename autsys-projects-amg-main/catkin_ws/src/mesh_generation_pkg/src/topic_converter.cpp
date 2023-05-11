/*This node is just for rostopic name converter, in oder to use the package stereo_image_pro.
We also need to change the camera parameters, due to the incorrection of camera_info topic.*/
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>

using namespace message_filters;

static const std::string tfrgbL = "Quadrotor/RGBCameraLeft";
static const std::string tfrgbR = "Quadrotor/RGBCameraRight";
static const std::string camera = "camera";

class topic_converter
{
    ros::NodeHandle nh_;
    message_filters::Subscriber<sensor_msgs::Image> image_left_sub;
    message_filters::Subscriber<sensor_msgs::Image> image_right_sub;
    TimeSynchronizer<sensor_msgs::Image,sensor_msgs::Image> *sync;

    ros::Publisher left_info;
    ros::Publisher left_image;
    ros::Publisher right_info;
    ros::Publisher right_image;
    ros::Subscriber left_info_sub;
    ros::Subscriber right_info_sub;

    boost::array<double,12> P, P1, P2;
    boost::array<double,9> K;

public:
  topic_converter()
  {
    image_left_sub.subscribe(nh_, "/realsense/rgb/left_image_raw", 1);
    image_right_sub.subscribe(nh_, "/realsense/rgb/right_image_raw", 1);
    sync = new TimeSynchronizer<sensor_msgs::Image,sensor_msgs::Image>(image_left_sub, image_right_sub, 1);
    sync->registerCallback(boost::bind(&topic_converter::callback, this, _1, _2));

    left_image = nh_.advertise<sensor_msgs::Image>("/stereo/left/image_raw", 1);
    right_image = nh_.advertise<sensor_msgs::Image>("/stereo/right/image_raw", 1);
    left_info = nh_.advertise<sensor_msgs::CameraInfo>("/stereo/left/camera_info", 1);
    right_info = nh_.advertise<sensor_msgs::CameraInfo>("/stereo/right/camera_info", 1);
    left_info_sub = nh_.subscribe("/realsense/rgb/left_image_info", 1, &topic_converter::callback2, this);
    right_info_sub = nh_.subscribe("/realsense/rgb/right_image_info", 1, &topic_converter::callback3, this);

    P1 = {160, 0.0, 160.0, 32, 0.0, 160, 120.0, 0.0, 0.0, 0.0, 1.0, 0.0};
    P2 = {160, 0.0, 160.0, 0, 0.0, 160, 120.0, 0.0, 0.0, 0.0, 1.0, 0.0};
    K = {111671.46926, 0.0, 160.0, 0.0, 111671.46926, 120, 0.0, 0.0, 1.0};
  }

  ~topic_converter(){}

  void callback2(const sensor_msgs::CameraInfo& info_left)
  {
    sensor_msgs::CameraInfo left_info_msg;
    left_info_msg.header.frame_id = camera;
    left_info_msg.header.seq = info_left.header.seq;
    left_info_msg.header.stamp = info_left.header.stamp;
    left_info_msg.height = info_left.height;
    left_info_msg.width = info_left.width;
    left_info_msg.distortion_model = info_left.distortion_model;
    left_info_msg.D = info_left.D;
    left_info_msg.K = K;
    left_info_msg.R = info_left.R;
    left_info_msg.P = P2;
    left_info_msg.binning_x = info_left.binning_x; 
    left_info_msg.binning_y = info_left.binning_y;
    left_info_msg.roi.x_offset = info_left.roi.x_offset;
    left_info_msg.roi.y_offset = info_left.roi.y_offset;
    left_info_msg.roi.height = info_left.roi.height;
    left_info_msg.roi.width = info_left.roi.width;
    left_info_msg.roi.do_rectify = info_left.roi.do_rectify;
    left_info.publish(left_info_msg);
  }

  void callback3(const sensor_msgs::CameraInfo& info_right)
  {
    sensor_msgs::CameraInfo right_info_msg;
    right_info_msg.header.frame_id = camera;
    right_info_msg.header.seq = info_right.header.seq;
    right_info_msg.header.stamp = info_right.header.stamp;
    right_info_msg.height = info_right.height;
    right_info_msg.width = info_right.width;
    right_info_msg.distortion_model = info_right.distortion_model;
    right_info_msg.D = info_right.D;
    right_info_msg.K = K;
    right_info_msg.R = info_right.R;
    right_info_msg.P = P1;
    right_info_msg.binning_x = info_right.binning_x; 
    right_info_msg.binning_y = info_right.binning_y;
    right_info_msg.roi.x_offset = info_right.roi.x_offset;
    right_info_msg.roi.y_offset = info_right.roi.y_offset;
    right_info_msg.roi.height = info_right.roi.height;
    right_info_msg.roi.width = info_right.roi.width;
    right_info_msg.roi.do_rectify = info_right.roi.do_rectify;
    right_info.publish(right_info_msg);
  }

  void callback(const sensor_msgs::ImageConstPtr& msg_left, const sensor_msgs::ImageConstPtr& msg_right)
  {
    left_image.publish(msg_left);
    right_image.publish(msg_right);
  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "topic_converter");
  topic_converter ld;
  ros::spin();
  return 0;
}
