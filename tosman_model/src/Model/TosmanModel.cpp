/*
 name : TosmanModel.cpp
 Author : Selçuk Ercan , M. Efe Tiryaki

 */

#include "tosman_model/Model/TosmanModel.hpp"

namespace kuco {

// Note : param_io is needed to use the getParam

TosmanModel::TosmanModel()
    : WheelL_(new Wheel()),
      WheelR_(new Wheel()),
      Wheels_(new std::vector<Wheel*>())
{
  Wheels_->push_back(WheelL_);
  Wheels_->push_back(WheelR_);
}

TosmanModel::~TosmanModel()
{
}

void TosmanModel::initilize()
{
}

void TosmanModel::create()
{
}

void TosmanModel::advance(double dt)
{
}

void TosmanModel::reset()
{
}

//void AracModel::setWheels(std::vector<Wheel>& Wheel)
//{
//  Wheels_= Wheel;
//}
//std::vector<Wheel>& AracModel::getWheels()
//{
//  return *Wheels_;
//}

std::vector<Wheel*>& TosmanModel::getWheel()
{
  return *Wheels_;
}

} /* namespace */
