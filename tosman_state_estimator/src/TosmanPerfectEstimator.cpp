#include "tosman_state_estimator/TosmanPerfectEstimator.hpp"

namespace estimator {

TosmanPerfectEstimator::TosmanPerfectEstimator(Model& model)
    : KulmanStateEstimatorBase(model)
{

}

void TosmanPerfectEstimator::initilize(ros::NodeHandle* nh)
{
  KulmanStateEstimatorBase::initilize(nh);

}

void TosmanPerfectEstimator::advance(double dt)
{
//  positionWorldToBase_[0] = kulmanStateMsg_.;
//  positionWorldToBase_[1] = kulmanStateMsg_.pose.pose.pos.y;
//  positionWorldToBase_[2] = kulmanStateMsg_.pose.pose.pos.z;
  positionWorldToBase_[0] = kulmanStateMsg_.pose.pose.position.x;
  positionWorldToBase_[1] = kulmanStateMsg_.pose.pose.position.y;
  positionWorldToBase_[2] = kulmanStateMsg_.pose.pose.position.z;

  orientationWorldToBase_[0] = kulmanStateMsg_.pose.pose.orientation.w;
  orientationWorldToBase_[1] = kulmanStateMsg_.pose.pose.orientation.x;
  orientationWorldToBase_[2] = kulmanStateMsg_.pose.pose.orientation.y;
  orientationWorldToBase_[3] = kulmanStateMsg_.pose.pose.orientation.z;

  velocityWorldToBaseInWorldFrame_[0] = kulmanStateMsg_.twist.twist.linear.x;
  velocityWorldToBaseInWorldFrame_[1] = kulmanStateMsg_.twist.twist.linear.y;
  velocityWorldToBaseInWorldFrame_[2] = kulmanStateMsg_.twist.twist.linear.z;

  angularVelocityWorldToBaseInWorldFrame_[0] = kulmanStateMsg_.twist.twist.angular.x;
  angularVelocityWorldToBaseInWorldFrame_[1] = kulmanStateMsg_.twist.twist.angular.y;
  angularVelocityWorldToBaseInWorldFrame_[2] = kulmanStateMsg_.twist.twist.angular.z;

  // Kestirilen konum,yönelim ve hızları ölçülen duruma ata.
  model_.getBody().getMeasuredState().setPositionInWorldFrame(positionWorldToBase_);
  model_.getBody().getMeasuredState().setOrientationInWorldFrame(orientationWorldToBase_);
  model_.getBody().getMeasuredState().setVelocityInWorldFrame(velocityWorldToBaseInWorldFrame_);
  model_.getBody().getMeasuredState().setAngularVelocityInWorldFrame(
      angularVelocityWorldToBaseInWorldFrame_);
}

}
