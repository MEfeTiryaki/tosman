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

#include "arac_model/State/State.hpp"
#include "tosman_model/Model/TosmanModel.hpp"

#include "kulman_state_estimator/KulmanStateEstimatorBase.hpp"

namespace estimator {

using Model = kuco::TosmanModel;

class TosmanPerfectEstimator : public KulmanStateEstimatorBase<Model>
{
 public:

  TosmanPerfectEstimator(Model& model);

  virtual void initilize(ros::NodeHandle* nh) override;

  virtual void advance(double dt) override;

 private:
  kuco::Velocity positionWorldToBase_;
  kuco::Quaternion orientationWorldToBase_;
  kuco::Velocity velocityWorldToBaseInWorldFrame_;
  kuco::AngularVelocity angularVelocityWorldToBaseInWorldFrame_;
};

}
