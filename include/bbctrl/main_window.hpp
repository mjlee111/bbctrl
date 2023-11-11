/**
 * @file /include/bbctrl/main_window.hpp
 *
 * @brief Qt based gui for %(package)s.
 *
 * @date November 2010
 **/
#ifndef bbctrl_MAIN_WINDOW_H
#define bbctrl_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include "qnode.hpp"
#include "ui_main_window.h"
#include <QImage>
#include <QString>>
#include <QMainWindow>
#include <fstream>
#include <sstream>
#include <string>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace bbctrl
{
/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(int argc, char** argv, QWidget* parent = 0);
  ~MainWindow();

  void updateSettingValue();
  void loadSettingValue();
  void saveSettingValue();

public Q_SLOTS:
  void imageUpdateSlot();
  void on_hsv_hH_valueChanged();
  void on_hsv_hL_valueChanged();
  void on_hsv_sH_valueChanged();
  void on_hsv_sL_valueChanged();
  void on_hsv_vH_valueChanged();
  void on_hsv_vL_valueChanged();
  void on_saveBtn_clicked();
  void on_loadBtn_clicked();

private:
  Ui::MainWindowDesign ui;
  QNode qnode;
};

}  // namespace bbctrl

#endif  // bbctrl_MAIN_WINDOW_H
