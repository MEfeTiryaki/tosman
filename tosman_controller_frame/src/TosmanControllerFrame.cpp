// arac gazebo
#include "tosman_controller_frame/TosmanControllerFrame.hpp"

namespace tosman_controller_frame {

// Note : param_io is needed to use the getParam
using namespace param_io;
TosmanControllerFrame::TosmanControllerFrame():
    loop_rate_(0)
{

}

TosmanControllerFrame::~TosmanControllerFrame()
{
}

void TosmanControllerFrame::create()
{
  //state_ = new kuco::State();
  model_ = new kuco::TosmanModel;
  estimator_ = new estimator::TosmanEKF(*model_);
  joystickHandler_ = new joystick::JoystickAcc<kuco::TosmanModel>(*model_);
  controller_ = new kuco::TosmanOLController(*model_);
}

void TosmanControllerFrame::initilize(int argc, char **argv)
{
  jointNames_.push_back("L_WH");
  jointNames_.push_back("R_WH");
  jointPositions_ = std::vector<double>(2, 0.0);
  jointVelocities_ = std::vector<double>(2, 0.0);
  jointEffort_ = std::vector<double>(2, 0.0);
  createActuatorCommand();

  nodeName_ = "/Tosman_controller_frame";

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

  std::cout << "Tosman_controller_frame::init " << std::endl;
}

void TosmanControllerFrame::update()
{

}

void TosmanControllerFrame::execute()
{
  while (ros::ok()) {
    advance();
    ros::spinOnce();
    loop_rate_->sleep();
  }
}

void TosmanControllerFrame::advance()
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

void TosmanControllerFrame::readParameters()
{
  // Get Publishers parameters
  getParam(*nodeHandle_, "publishers/actuator_commands/topic", actuatorCommandPublisherName_);
  getParam(*nodeHandle_, "publishers/actuator_commands/queue_size",
           actuatorCommandPublisherQueueSize_);

}

void TosmanControllerFrame::initilizePublishers()
{
  std::cout << "Tosman_controller_frame::initilizePublishers" << std::endl;
  actuatorCommandPublisher_ = nodeHandle_->advertise<arac_msgs::ActuatorCommands>(
      actuatorCommandPublisherName_, actuatorCommandPublisherQueueSize_);

}

void TosmanControllerFrame::initilizeSubscribers()
{

}

void TosmanControllerFrame::createActuatorCommand()
{
  actuatorCommand_.inputs.name = jointNames_;
  actuatorCommand_.inputs.position = jointVelocities_;
  actuatorCommand_.inputs.velocity = jointVelocities_;
  actuatorCommand_.inputs.effort = jointEffort_;

}

void TosmanControllerFrame::setActuatorCommand()
{
  double wheelVelocities;
  auto& tekerler = model_->getTekerlek();
  for (int i = 0; i < actuatorCommand_.inputs.velocity.size(); i++) {
    wheelVelocities = tekerler[i]->getDesiredState().getAngularVelocityInWorldFrame()[2];
    actuatorCommand_.inputs.velocity[i] = wheelVelocities;
  }
}

} /* namespace tosman_controller_frame*/
