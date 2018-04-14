#include <arac2_controller/Arac2OLController.hpp>
#include <param_io/get_param.hpp>

namespace kuco {

// Note : param_io is needed to use the getParam
using namespace param_io;

// Todo : make controllerInput length adjustable
Arac2OLController::Arac2OLController(Model& model)
    : LinearOpenLoopController<Model>(model),
      rWheel_(0),
      lWheel_(0)
{
}

Arac2OLController::~Arac2OLController()
{
}

void Arac2OLController::initilize()
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

  // Todo : arac icin B yi tanimla
  B_.setZero(l_, n_);
  B_.row(0) << -rWheel_ / 2, -rWheel_ / 2;
  B_.row(1) << rWheel_ / 2 / lWheel_, -rWheel_ / 2 / lWheel_;
}

void Arac2OLController::readParameters()
{

}

void Arac2OLController::getState()
{
  // Todo : Burayi Kulman konacligina cevirmek gerekiyor
  x_[0] = model_.getGovde().getDesiredState().getVelocityInWorldFrame()[0];
  x_[1] = model_.getGovde().getDesiredState().getAngularVelocityInWorldFrame()[2];
}

void Arac2OLController::setActuatorCommand()
{
  std::vector<Tekerlek*>& wheels = model_.getTekerlek();
  kuco::AngularVelocity input;

  // L
  input << 0.0, 0.0, u_[0];
  wheels[0]->getDesiredState().setAngularVelocityInWorldFrame(input);
  // R
  input << 0.0, 0.0, u_[1];
  wheels[1]->getDesiredState().setAngularVelocityInWorldFrame(input);
}

} /* namespace kuco*/
