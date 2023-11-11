/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QApplication>
#include "../include/bbctrl/main_window.hpp"

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char** argv)
{
  /*********************
  ** Qt
  **********************/
  QApplication app(argc, argv);
  bbctrl::MainWindow w(argc, argv);
  w.setStyleSheet("background-color: gray;");
  w.show();
  app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
  int result = app.exec();

  return result;
}
