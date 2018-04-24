/*
 name : TosmanModel.hpp
 Author : Selçuk Ercan , M. Efe Tiryaki

 */

#pragma once

#include "kulman_model/Model/KulmanModel.hpp"
#include "kulman_model/Module/Wheel.hpp"
#include <vector>

namespace kuco {

class TosmanModel : public KulmanModel
{
 public:
  TosmanModel();

  virtual ~TosmanModel();

  virtual void initilize();

  virtual void create();

  virtual void advance(double dt);

  virtual void reset();

  //void setWheel(Wheel Wheel);

  std::vector<Wheel*>& getWheel();


 protected:

  // vector içinde pointer ne kadar doğru bir yaklaşım emin değilim ama dursun
  Wheel* WheelL_;
  Wheel* WheelR_;

  std::vector<Wheel*>* Wheels_;
};

} /* namespace kuco */
