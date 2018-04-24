#pragma once

#include <ros/ros.h>

// ROS messages / services
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Empty.h>

#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <memory>
#include <mutex>
#include <vector>

#include <param_io/get_param.hpp>

#include "kulman_controller_frame/KulmanControllerFrame.hpp"
#include "kulman_joystick/JoystickAcc.hpp"
#include "tosman_controller/TosmanOLController.hpp"
#include "kulman_msgs/ActuatorCommands.h"
#include "tosman_model/TosmanModel.hpp"
#include "tosman_state_estimator/TosmanStateEstimatorHandler.hpp"

// stl
#include <memory>

namespace kuco {

using KulmanModel_ = kuco::TosmanModel;
using Controller_ = kuco::TosmanOLController;
using EstimatorHandler_ = estimator::TosmanStateEstimatorHandler;
using Joystick_ = joystick::JoystickAcc<KulmanModel_>;

class TosmanControllerFrame : public KulmanControllerFrame<KulmanModel_,Controller_,EstimatorHandler_,Joystick_>
{
 public:
  // Constructor.
  TosmanControllerFrame();

  // Destructor.
  virtual ~TosmanControllerFrame();

  // Init
  virtual void initilize(int argc, char **argv) override;

  virtual void create() override ;

 protected:

  void setActuatorCommand() override;

};

}
