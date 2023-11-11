/**
 * @file /include/bbctrl/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef bbctrl_QNODE_HPP_
#define bbctrl_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include "image_transport/image_transport.h"
#include <cv_bridge/cv_bridge.h>
#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "sensor_msgs/Image.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace bbctrl
{
/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread
{
  Q_OBJECT
public:
  QNode(int argc, char** argv);
  virtual ~QNode();
  bool init();
  void run();

  cv::Mat* raw_img;
  cv::Mat resize_img;
  cv::Mat binary_img;

  int hsv_value[6] = {
    0,
  };

Q_SIGNALS:
  void rosShutdown();
  void imgSignalEmit();

private:
  int init_argc;
  char** init_argv;

  image_transport::Subscriber img_subscriber;
  ros::Publisher cmd_vel_publisher;

  void imgCallback(const sensor_msgs::ImageConstPtr& img_raw);
  void changeToBinary(cv::Mat& input_img, cv::Mat& output_img, int hsv_value[]);
};

}  // namespace bbctrl

#endif /* bbctrl_QNODE_HPP_ */
