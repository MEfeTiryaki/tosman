#include <ros/ros.h>
#include <iostream>

#include "tosman_controller_frame/TosmanControllerFrame.hpp"


int main(int argc, char **argv)
{
  tosman_controller_frame::TosmanControllerFrame controllerFrame = tosman_controller_frame::TosmanControllerFrame();
  controllerFrame.create();
  controllerFrame.initilize(argc,argv);
  controllerFrame.execute();
  return 0;
}
