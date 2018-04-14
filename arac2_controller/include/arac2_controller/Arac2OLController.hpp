#pragma once

#include "arac_model/Model/Arac2Model.hpp"
#include "arac_controller/LinearOpenLoopController.hpp"
#include <vector>

namespace kuco {

using Model = kuco::Arac2Model;
class Arac2OLController : public LinearOpenLoopController<Model>
{
 public:
  // Constructor.
//  aracController(kuco::State& state);
  Arac2OLController(Model& model);

  // Destructor.
  virtual ~Arac2OLController();

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
