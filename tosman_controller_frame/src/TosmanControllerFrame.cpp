// tosman gazebo
#include "tosman_controller_frame/TosmanControllerFrame.hpp"

namespace kuco {

// Note : param_io is needed to use the getParam
using namespace param_io;
TosmanControllerFrame::TosmanControllerFrame():
    KulmanControllerFrame<KulmanModel_,Controller_,Estimator_,Joystick_>()
{

}

TosmanControllerFrame::~TosmanControllerFrame()
{
}

void TosmanControllerFrame::create()
{
  this->model_ = new kuco::TosmanModel;
  this->estimator_ = new estimator::TosmanEKF(*model_);
  this->joystickHandler_ = new joystick::JoystickAcc<kuco::TosmanModel>(*model_);
  this->controller_ = new kuco::TosmanOLController(*model_);
}

void TosmanControllerFrame::initilize(int argc, char **argv)
{
  // nodeHandler olusturuldu.
  this->nodeName_ = "/tosman_controller_frame";
  ros::init(argc, argv, nodeName_);
  this->nodeHandle_ = new ros::NodeHandle("~");

  // Parametreler okundu.
  readParameters();

  // loop rate ayarlandi
  this->loop_rate_ = new ros::Rate(loopFrequency_);

  this->jointPositions_ = std::vector<double>(n_, 0.0);
  this->jointVelocities_ = std::vector<double>(n_, 0.0);
  this->jointEffort_ = std::vector<double>(n_, 0.0);
  createActuatorCommand();

  initilizePublishers();
  initilizeSubscribers();

  this->model_->initilize();
  this->joystickHandler_->initilize(nodeHandle_);
  this->estimator_->initilize(nodeHandle_);
  this->controller_->initilize();

  print();
}

void TosmanControllerFrame::setActuatorCommand()
{
  double wheelVelocities;
  auto& wheels = this->model_->getWheel();
  for (int i = 0; i < actuatorCommand_.inputs.velocity.size(); i++) {
    wheelVelocities = wheels[i]->getDesiredState().getAngularVelocityInWorldFrame()[2];
    this->actuatorCommand_.inputs.velocity[i] = wheelVelocities;
  }
}


} /* namespace kuco*/
