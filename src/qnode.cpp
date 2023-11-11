/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/bbctrl/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace bbctrl
{
/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv) : init_argc(argc), init_argv(argv)
{
}

QNode::~QNode()
{
  if (ros::isStarted())
  {
    ros::shutdown();  // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

bool QNode::init()
{
  ros::init(init_argc, init_argv, "bbctrl");
  if (!ros::master::check())
  {
    return false;
  }
  ros::start();  // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;
  package_path = ros::package::getPath("bbctrl");
  hsv_value_path = package_path + "/image_value_data";
  ROS_INFO("Current ROS Path is %s", package_path.c_str());
  ROS_INFO("Loading HSV value from Path %s", hsv_value_path.c_str());

  // Add your ros communications here.
  image_transport::ImageTransport it(n);
  img_subscriber = it.subscribe("/usb_cam/image_raw", 1, &QNode::imgCallback, this);
  start();
  return true;
}

void QNode::run()
{
  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown();  // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::imgCallback(const sensor_msgs::ImageConstPtr& img_raw)
{
  raw_img = new cv::Mat(cv_bridge::toCvCopy(img_raw, sensor_msgs::image_encodings::BGR8)->image);
  resize_img = raw_img->clone();
  cv::resize(resize_img, resize_img, cv::Size(320, 240), 0, 0, CV_INTER_LINEAR);
  changeToBinary(resize_img, binary_img, hsv_value);
  path_img = resize_img.clone();
  path_img = segmentAndOverlay(path_img, binary_img);
  cv::resize(path_img, path_img, cv::Size(640, 480), 0, 0, CV_INTER_LINEAR);
  Q_EMIT imgSignalEmit();
}

void QNode::changeToBinary(cv::Mat& input_img, cv::Mat& output_img, int hsv_value[])
{
  if (input_img.empty())
  {
    return;
  }

  cv::Mat hsv_img;
  cv::cvtColor(input_img, hsv_img, cv::COLOR_BGR2HSV);
  cv::medianBlur(hsv_img, hsv_img, 9);
  cv::GaussianBlur(hsv_img, hsv_img, cv::Size(15, 15), 2.0);

  cv::Scalar lower_bound(hsv_value[1], hsv_value[3], hsv_value[5]);
  cv::Scalar upper_bound(hsv_value[0], hsv_value[2], hsv_value[4]);

  // Threshold the image based on HSV values
  cv::inRange(hsv_img, lower_bound, upper_bound, output_img);
  cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(-1, -1));
  cv::erode(output_img, output_img, element);
  cv::dilate(output_img, output_img, element);
  cv::resize(output_img, output_img, cv::Size(320, 240), 0, 0, cv::INTER_LINEAR);
}

cv::Mat QNode::segmentAndOverlay(const cv::Mat& originalImage, const cv::Mat& binaryImage)
{
  // Find contours in the binary image
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binaryImage, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  // Find the contour with the largest area
  double maxArea = 0;
  int maxAreaIdx = -1;
  for (int i = 0; i < contours.size(); ++i)
  {
    double area = cv::contourArea(contours[i]);
    if (area > maxArea)
    {
      maxArea = area;
      maxAreaIdx = i;
    }
  }

  // Create a mask for the largest contour
  cv::Mat mask = cv::Mat::zeros(binaryImage.size(), CV_8U);
  if (maxAreaIdx != -1)
  {
    cv::drawContours(mask, contours, maxAreaIdx, cv::Scalar(255), cv::FILLED);
  }

  // Create an output image with the same size and type as the original image
  cv::Mat outputImage = originalImage.clone();

  // Draw a semi-transparent red overlay on the original image based on the mask
  cv::Mat overlay = cv::Mat::zeros(originalImage.size(), originalImage.type());
  overlay.setTo(cv::Scalar(0, 0, 255), mask);
  cv::addWeighted(originalImage, 1.0, overlay, 0.5, 0, outputImage);
  return outputImage;
}

}  // namespace bbctrl
