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
  // Add your ros communications here.
  image_transport::ImageTransport it(n);
  img_subscriber = it.subscribe("/usb_cam/image_raw", 1, &QNode::imgCallback, this);
  start();
  return true;
}

void QNode::run()
{
  ros::Rate loop_rate(33);
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

}  // namespace bbctrl

// if (input_img.empty())
// {
//   return;
// }

// cv::Mat erode_kernel = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(-1, -1));
// cv::Mat dilate_kernel = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(-1, -1));

// output_img = input_img.clone();

// medianBlur(output_img, output_img, 9);
// GaussianBlur(output_img, output_img, cv::Size(15, 15), 2.0);

// cvtColor(output_img, output_img, cv::COLOR_BGR2HSV);

// if (hsv_value[0] != 255)
// {
//   cv::inRange(output_img, cv::Scalar(hsv_value[3], hsv_value[4], hsv_value[5]),
//               cv::Scalar(hsv_value[0], hsv_value[1], hsv_value[2]), output_img);
// }
// else
// {
//   cv::Mat mask1, mask2;
//   cv::inRange(output_img, cv::Scalar(0, hsv_value[4], hsv_value[5]),
//               cv::Scalar(hsv_value[2], hsv_value[1], hsv_value[2]), mask1);
//   cv::inRange(output_img, cv::Scalar(hsv_value[3], hsv_value[4], hsv_value[5]),
//               cv::Scalar(255, hsv_value[1], hsv_value[2]), mask2);
//   cv::bitwise_or(mask1, mask2, output_img);
// }

// cv::erode(output_img, output_img, erode_kernel);
// cv::dilate(output_img, output_img, dilate_kernel);