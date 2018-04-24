#include <tosman_controller/TosmanOLController.hpp>
#include <param_io/get_param.hpp>

namespace kuco {

// Note : param_io is needed to use the getParam
using namespace param_io;

// Todo : make controllerInput length adjustable
TosmanOLController::TosmanOLController(Model& model)
    : LinearOpenLoopController<Model>(model),
      rWheel_(0),
      lWheel_(0)
{
}

TosmanOLController::~TosmanOLController()
{
}

void TosmanOLController::initilize()
{
  // Durum uzayi eleman sayisi
  n_ = 2;
  // Girdi sayisi
  l_ = 2;

  //
  rWheel_ = 0.15;
  lWheel_ = 0.2;

  //
  x_.setZero(n_);
  u_.setZero(l_);

  // Todo : tosman icin B yi tanimla
  B_.setZero(l_, n_);
  B_.row(0) << -rWheel_ / 2, -rWheel_ / 2;
  B_.row(1) << rWheel_ / 2 / lWheel_, -rWheel_ / 2 / lWheel_;
}

void TosmanOLController::readParameters()
{

}

void TosmanOLController::getState()
{
  // Todo : Burayi Kulman konacligina cevirmek gerekiyor
  x_[0] = model_.getBody().getDesiredState().getVelocityInWorldFrame()[0];
  x_[1] = model_.getBody().getDesiredState().getAngularVelocityInWorldFrame()[2];
}

void TosmanOLController::setActuatorCommand()
{
  std::vector<Wheel*>& wheels = model_.getWheel();
  kuco::AngularVelocity input;

  // L
  input << 0.0, 0.0, u_[0];
  wheels[0]->getDesiredState().setAngularVelocityInWorldFrame(input);
  // R
  input << 0.0, 0.0, u_[1];
  wheels[1]->getDesiredState().setAngularVelocityInWorldFrame(input);
}

} /* namespace kuco*/
