// arac gazebo
#include "arac2_controller_frame/arac2ControllerFrame.hpp"

namespace arac2_controller_frame {

// Note : param_io is needed to use the getParam
using namespace param_io;
arac2ControllerFrame::arac2ControllerFrame():
    loop_rate_(0)
{

}

arac2ControllerFrame::~arac2ControllerFrame()
{
}

void arac2ControllerFrame::create()
{
  //state_ = new kuco::State();
  model_ = new kuco::AracModel;
  estimator_ = new estimator::AracEKF(*model_);
  joystickHandler_ = new joystick::JoystickAcc(*model_);
  controller_ = new kuco::AracOLController(*model_);
}

void arac2ControllerFrame::initilize(int argc, char **argv)
{
  jointNames_.push_back("L_WH");
  jointNames_.push_back("R_WH");
  jointPositions_ = std::vector<double>(2, 0.0);
  jointVelocities_ = std::vector<double>(2, 0.0);
  jointEffort_ = std::vector<double>(2, 0.0);
  createActuatorCommand();

  nodeName_ = "/arac2_controller_frame";

  ros::init(argc, argv, nodeName_);

  nodeHandle_ = new ros::NodeHandle("~");

  loop_rate_ = new ros::Rate(100);

  readParameters();
  initilizePublishers();
  initilizeSubscribers();

  model_->initilize();
  joystickHandler_->initilize(nodeHandle_);
  estimator_->initilize(nodeHandle_);
  controller_->initilize();

  std::cout << "arac2_controller_frame::init " << std::endl;
}

void arac2ControllerFrame::update()
{

}

void arac2ControllerFrame::execute()
{
  while (ros::ok()) {
    advance();
    ros::spinOnce();
    loop_rate_->sleep();
  }
}

void arac2ControllerFrame::advance()
{

  // Estimator here in future
  estimator_->advance();

  // Advance the joystick handler
  joystickHandler_->advance();

  // Advance the controller
  controller_->advance();

  // set actuator commands
  setActuatorCommand();
  // publish actuators
  actuatorCommandPublisher_.publish(actuatorCommand_);

}

void arac2ControllerFrame::readParameters()
{
  // Get Publishers parameters
  getParam(*nodeHandle_, "publishers/actuator_commands/topic", actuatorCommandPublisherName_);
  getParam(*nodeHandle_, "publishers/actuator_commands/queue_size",
           actuatorCommandPublisherQueueSize_);

}

void arac2ControllerFrame::initilizePublishers()
{
  std::cout << "arac2_controller_frame::initilizePublishers" << std::endl;
  actuatorCommandPublisher_ = nodeHandle_->advertise<arac_msgs::ActuatorCommands>(
      actuatorCommandPublisherName_, actuatorCommandPublisherQueueSize_);

}

void arac2ControllerFrame::initilizeSubscribers()
{

}

void arac2ControllerFrame::createActuatorCommand()
{
  actuatorCommand_.inputs.name = jointNames_;
  actuatorCommand_.inputs.position = jointVelocities_;
  actuatorCommand_.inputs.velocity = jointVelocities_;
  actuatorCommand_.inputs.effort = jointEffort_;

}

void arac2ControllerFrame::setActuatorCommand()
{
  double wheelVelocities;
  auto& tekerler = model_->getTekerlek();
  for (int i = 0; i < actuatorCommand_.inputs.velocity.size(); i++) {
    wheelVelocities = tekerler[i]->getDesiredState().getAngularVelocityInWorldFrame()[2];
    actuatorCommand_.inputs.velocity[i] = wheelVelocities;
  }
}

} /* namespace arac2ControllerFrame*/
