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
#include "arac_controller/AracOLController.hpp"
#include "arac_msgs/ActuatorCommands.h"
#include "arac_model/State/State.hpp"
#include "arac_model/Model/Arac2Model.hpp"
#include "arac_state_estimator/AracEKF.hpp"

// stl
#include <memory>

namespace arac2_controller_frame {

class arac2ControllerFrame
{
 public:
  // Constructor.
  arac2ControllerFrame();

  // Destructor.
  virtual ~arac2ControllerFrame();

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

  estimator::AracEKF* estimator_;
  joystick::JoystickAcc* joystickHandler_ ;
  kuco::AracOLController* controller_ ;
  kuco::State* state_;
  kuco::AracModel* model_;


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
