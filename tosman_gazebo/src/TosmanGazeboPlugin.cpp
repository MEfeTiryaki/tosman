/*
 Author : M. Efe Tiryaki
 */

// param io
#include <param_io/get_param.hpp>

// tosman gazebo
#include "tosman_gazebo/TosmanGazeboPlugin.hpp"

namespace gazebo {

// Note : param_io is needed to use the getParam
using namespace param_io;
TosmanGazeboPlugin::TosmanGazeboPlugin()
    : KulmanGazeboPlugin()
{
}

TosmanGazeboPlugin::~TosmanGazeboPlugin()
{

}


void TosmanGazeboPlugin::readSimulation(){
  KulmanGazeboPlugin::readSimulation();
  // read joint angles and write in publisher
  for (int i = 0 ; i < jointPtrs_.size() ;i++) {
    const auto jointPtr = jointPtrs_[i];
    jointStates_.position[i] = jointPtr->GetAngle(0).Radian();
    jointStates_.velocity[i] = jointPtr->GetVelocity(0);
    jointStates_.effort[i] = jointPtr->GetForce(0);
  }
  kulmanStateMsg_.pose.pose.position.x = baseLink_->GetWorldPose().pos.x ;
  kulmanStateMsg_.pose.pose.position.y = baseLink_->GetWorldPose().pos.y ;
  kulmanStateMsg_.pose.pose.position.z = baseLink_->GetWorldPose().pos.z ;

  kulmanStateMsg_.pose.pose.orientation.w = baseLink_->GetWorldPose().rot.w ;
  kulmanStateMsg_.pose.pose.orientation.x = baseLink_->GetWorldPose().rot.x ;
  kulmanStateMsg_.pose.pose.orientation.y = baseLink_->GetWorldPose().rot.y ;
  kulmanStateMsg_.pose.pose.orientation.z = baseLink_->GetWorldPose().rot.z ;

  kulmanStateMsg_.twist.twist.linear.x = baseLink_->GetWorldLinearVel().x ;
  kulmanStateMsg_.twist.twist.linear.y = baseLink_->GetWorldLinearVel().y ;
  kulmanStateMsg_.twist.twist.linear.z = baseLink_->GetWorldLinearVel().z ;

  kulmanStateMsg_.twist.twist.angular.x = baseLink_->GetWorldAngularVel().x ;
  kulmanStateMsg_.twist.twist.angular.y = baseLink_->GetWorldAngularVel().y ;
  kulmanStateMsg_.twist.twist.angular.z = baseLink_->GetWorldAngularVel().z ;

}

void TosmanGazeboPlugin::writeSimulation()
{
  // TODO : 2 den kurtul adam gibi ekle bunu
  for (int i = 0 ;i<2 ;i++ ) {
    double jointVelocity = actuatorCommands_.inputs.velocity[i];
    jointPtrs_[i]->SetVelocity(0, jointVelocity);
  }
}

void TosmanGazeboPlugin::initPublishers()
{
  KulmanGazeboPlugin::initPublishers();

  if (isEstimatorUsed) {
    // Todo : implement Imu data for state estimation
    imuDataPublisher_ = nodeHandle_->advertise<sensor_msgs::Imu>(robotName_ + "/ImuData", 1);
    // Todo : implement joint data for state estimation
    actuatorDataPublisher_ = nodeHandle_->advertise<sensor_msgs::JointState>(
        robotName_ + "/ActuatorData", 1);
  }
}

void TosmanGazeboPlugin::initSubscribers()
{
  KulmanGazeboPlugin::initSubscribers();
}


GZ_REGISTER_MODEL_PLUGIN(TosmanGazeboPlugin)
}
