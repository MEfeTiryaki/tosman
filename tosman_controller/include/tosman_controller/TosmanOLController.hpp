#pragma once

#include "tosman_model/TosmanModel.hpp"
#include "kulman_controller/LinearOpenLoopController.hpp"
#include <vector>

namespace kuco {

using Model = kuco::TosmanModel;
class TosmanOLController : public LinearOpenLoopController<Model>
{
 public:
  // Constructor.
//  tosmanController(kuco::State& state);
  TosmanOLController(Model& model);

  // Destructor.
  virtual ~TosmanOLController();

  // Init
  virtual void initilize() override;

  // Parameters init
  virtual void readParameters() override;

 protected:

  virtual void getState() override;

  virtual void setActuatorCommand() override;

  // Wheel yaricapi
  double rWheel_ ;
  // Wheel uzakligi
  double lWheel_ ;
};

} /* kuco adligi*/
