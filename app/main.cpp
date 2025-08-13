#include "Picking_Studio/mainwindow.hpp"
#include <QApplication>
#include <ros/ros.h>

int main(int argc, char **argv) {
  QApplication app(argc, argv);
  ros::init(argc, argv, "Picking_Studio_Node");
  MainWindow mainWindow;
  mainWindow.show();
  app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
  int result = app.exec();

  return result;
}
