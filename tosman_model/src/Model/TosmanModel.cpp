/*
 name : TosmanModel.cpp
 Author : Selçuk Ercan , M. Efe Tiryaki

 */

#include "tosman_model/Model/TosmanModel.hpp"

namespace kuco {

// Note : param_io is needed to use the getParam

TosmanModel::TosmanModel()
    : tekerlekL_(new Tekerlek()),
      tekerlekR_(new Tekerlek()),
      tekerlekler_(new std::vector<Tekerlek*>())
{
  tekerlekler_->push_back(tekerlekL_);
  tekerlekler_->push_back(tekerlekR_);
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

void TosmanModel::advance()
{
}

void TosmanModel::reset()
{
}

//void AracModel::setTekerlekler(std::vector<Tekerlek>& tekerlek)
//{
//  tekerlekler_= tekerlek;
//}
//std::vector<Tekerlek>& AracModel::getTekerlekler()
//{
//  return *tekerlekler_;
//}

std::vector<Tekerlek*>& TosmanModel::getTekerlek()
{
  return *tekerlekler_;
}

} /* namespace */
