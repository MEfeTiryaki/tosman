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

#include "arac_joystick/JoystickAcc.hpp"
#include "tosman_controller/TosmanOLController.hpp"
#include "arac_msgs/ActuatorCommands.h"
#include "arac_model/State/State.hpp"
#include "tosman_model/Model/TosmanModel.hpp"
#include "tosman_state_estimator/TosmanEKF.hpp"

// stl
#include <memory>

namespace tosman_controller_frame {

class TosmanControllerFrame
{
 public:
  // Constructor.
  TosmanControllerFrame();

  // Destructor.
  virtual ~TosmanControllerFrame();

  // Init
  virtual void initilize(int argc, char **argv);

  // Create
  virtual void create();

  // Parameters init
  virtual void readParameters();

  // Update
  virtual void update();

  // excute
  virtual void execute();

  // excute
  virtual void advance();

 protected:


  virtual void initilizePublishers();

  virtual void initilizeSubscribers();

  virtual void setActuatorCommand();

  void createActuatorCommand();

 private:

  ros::NodeHandle* nodeHandle_;

  ros::Rate* loop_rate_;

  std::string nodeName_;
  std::string robotName_;

  estimator::TosmanEKF* estimator_;
  joystick::JoystickAcc<kuco::TosmanModel>* joystickHandler_ ;
  kuco::TosmanOLController* controller_ ;
  kuco::State* state_;
  kuco::TosmanModel* model_;


  // Publisher
  ros::Publisher actuatorCommandPublisher_;
  // Publisher names
  std::string actuatorCommandPublisherName_;
  // Publisher queue_size
  int actuatorCommandPublisherQueueSize_;
  // Publisher msgs
  arac_msgs::ActuatorCommands actuatorCommand_;


  double joystickCommandStartTime_;



  std::vector<std::string> jointNames_;
  std::vector<double> jointPositions_;
  std::vector<double> jointVelocities_;
  std::vector<double> jointEffort_;


};

}
