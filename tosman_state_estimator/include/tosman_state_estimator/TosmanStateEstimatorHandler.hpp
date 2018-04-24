#include <ros/ros.h>

// ROS messages / services
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Empty.h>

#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <memory>
#include <mutex>

#include "tosman_model/TosmanModel.hpp"
#include "kulman_state_estimator/StateEstimatorHandlerBase.hpp"

#include <param_io/get_param.hpp>

namespace estimator {

using namespace param_io;

using Model_ = kuco::TosmanModel;

class TosmanStateEstimatorHandler : public  StateEstimatorHandlerBase<Model_>
{
 public:
  TosmanStateEstimatorHandler(Model_& model);

  virtual ~TosmanStateEstimatorHandler(){}

  virtual void advance(double dt) override;


protected:
  kuco::Velocity positionWorldToBase_;
  kuco::Quaternion orientationWorldToBase_;
  kuco::Velocity velocityWorldToBaseInWorldFrame_;
  kuco::AngularVelocity angularVelocityWorldToBaseInWorldFrame_;
};

}
