#pragma once

#include "tosman_model/Model/TosmanModel.hpp"
#include "arac_controller/LinearOpenLoopController.hpp"
#include <vector>

namespace kuco {

using Model = kuco::TosmanModel;
class TosmanOLController : public LinearOpenLoopController<Model>
{
 public:
  // Constructor.
//  aracController(kuco::State& state);
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

  // tekerlek yaricapi
  double rWheel_ ;
  // tekerlek uzakligi
  double lWheel_ ;
};

} /* kuco adligi*/
