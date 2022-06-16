#include <QApplication>
#include "ornibibotgui.h"

int main(int argc, char *argv[]){

  ros::init(argc, argv, "Hello gui");
  QApplication a (argc, argv);

  OrnibiBotGUI w;

  w.setWindowTitle(QString::fromStdString(
                     ros::this_node::getName()));
  w.show();
  return a.exec();

}
