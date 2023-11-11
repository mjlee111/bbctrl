/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/bbctrl/main_window.hpp"
#include <QMessageBox>
#include <QtGui>
#include <iostream>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace bbctrl
{
using namespace Qt;
/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget* parent) : QMainWindow(parent), qnode(argc, argv)
{
  ui.setupUi(this);  // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

  setWindowIcon(QIcon(":/images/icon.png"));

  qnode.init();

  updateSettingValue();
  ui.package_path->setText(QString(qnode.package_path.c_str()));
  loadSettingValue();

  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
  QObject::connect(&qnode, SIGNAL(imgSignalEmit()), this, SLOT(imageUpdateSlot()));
}

MainWindow::~MainWindow()
{
}

/*****************************************************************************
** Functions
*****************************************************************************/

void MainWindow::imageUpdateSlot()
{
  QImage qraw_img((const unsigned char*)(qnode.resize_img.data), qnode.resize_img.cols, qnode.resize_img.rows,
                  QImage::Format_RGB888);
  ui.img->setPixmap(QPixmap::fromImage(qraw_img.rgbSwapped()));
  QImage qbinary_img((const unsigned char*)(qnode.binary_img.data), qnode.binary_img.cols, qnode.binary_img.rows,
                     QImage::Format_Grayscale8);
  ui.binary->setPixmap(QPixmap::fromImage(qbinary_img.rgbSwapped()));
  QImage qpath_img((const unsigned char*)(qnode.path_img.data), qnode.path_img.cols, qnode.path_img.rows,
                   QImage::Format_RGB888);
  ui.path->setPixmap(QPixmap::fromImage(qpath_img.rgbSwapped()));

  delete qnode.raw_img;
}

void MainWindow::updateSettingValue()
{
  qnode.hsv_value[0] = ui.hsv_hH->value();
  qnode.hsv_value[1] = ui.hsv_hL->value();
  qnode.hsv_value[2] = ui.hsv_sH->value();
  qnode.hsv_value[3] = ui.hsv_sL->value();
  qnode.hsv_value[4] = ui.hsv_vH->value();
  qnode.hsv_value[5] = ui.hsv_vL->value();

  ui.hH->setText(QString::number(ui.hsv_hH->value()));
  ui.hL->setText(QString::number(ui.hsv_hL->value()));
  ui.sH->setText(QString::number(ui.hsv_sH->value()));
  ui.sL->setText(QString::number(ui.hsv_sL->value()));
  ui.vH->setText(QString::number(ui.hsv_vH->value()));
  ui.vL->setText(QString::number(ui.hsv_vL->value()));
}

void MainWindow::loadSettingValue()
{
  std::string filename = qnode.hsv_value_path + "/data.txt";
  std::ifstream file(filename);

  if (file.is_open())
  {
    for (int i = 0; i < 6; ++i)
    {
      std::string line;
      if (std::getline(file, line))
      {
        std::istringstream iss(line);
        iss >> qnode.hsv_value[i];
      }
      else
      {
        std::cerr << "Unexpected end of file: " << filename << std::endl;
        break;
      }
    }
    file.close();
    std::cout << "HSV values loaded from file: " << filename << std::endl;
  }
  else
  {
    std::cerr << "Unable to open file: " << filename << std::endl;
  }
}

void MainWindow::saveSettingValue()
{
  std::string filename = qnode.hsv_value_path + "/data.txt";
  std::ofstream file(filename);

  if (file.is_open())
  {
    for (int i = 0; i < 6; ++i)
    {
      file << qnode.hsv_value[i] << '\n';
    }
    file.close();
    std::cout << "HSV values saved to file: " << filename << std::endl;
  }
  else
  {
    std::cerr << "Unable to open file: " << filename << std::endl;
  }
}

void MainWindow::on_saveBtn_clicked()
{
  saveSettingValue();
}

void MainWindow::on_loadBtn_clicked()
{
  loadSettingValue();
}

void MainWindow::on_hsv_hH_valueChanged()
{
  updateSettingValue();
}

void MainWindow::on_hsv_hL_valueChanged()
{
  updateSettingValue();
}

void MainWindow::on_hsv_sH_valueChanged()
{
  updateSettingValue();
}

void MainWindow::on_hsv_sL_valueChanged()
{
  updateSettingValue();
}

void MainWindow::on_hsv_vH_valueChanged()
{
  updateSettingValue();
}

void MainWindow::on_hsv_vL_valueChanged()
{
  updateSettingValue();
}
}  // namespace bbctrl
