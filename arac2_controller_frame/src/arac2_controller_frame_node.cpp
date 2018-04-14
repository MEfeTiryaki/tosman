#include <ros/ros.h>
#include <iostream>

#include "arac2_controller_frame/arac2ControllerFrame.hpp"


int main(int argc, char **argv)
{
  arac2_controller_frame::arac2ControllerFrame controllerFrame = arac2_controller_frame::arac2ControllerFrame();
  controllerFrame.create();
  controllerFrame.initilize(argc,argv);
  controllerFrame.execute();
  return 0;
}
