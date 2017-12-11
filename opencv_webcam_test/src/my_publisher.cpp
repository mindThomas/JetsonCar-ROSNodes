#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  ROS_INFO("Launching node");

  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);
  //cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR); // load image based on input argument
  cv::Mat frame; 
  cv::VideoCapture cap("v4l2src device=\"/dev/video1\" name=e ! video/x-raw, width=1920, height=1080, format=(string)BGR ! videoconvert ! appsink"); // open the default camera
  cv::waitKey(30);
  sensor_msgs::ImagePtr msg;

  //ros::Rate loop_rate(5);
  while (nh.ok()) {
    cap >> frame; // get a new frame from camera
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    ROS_INFO("Publishing");
    pub.publish(msg);
    ros::spinOnce();
    //loop_rate.sleep();
  }
}
