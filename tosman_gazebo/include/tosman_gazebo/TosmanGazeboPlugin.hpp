/*
 Author : M. Efe Tiryaki
 */

#pragma once

// TODO : bunu kulmana gecir

#include "arac_gazebo/KulmanGazeboPlugin.hpp"

namespace gazebo {
class TosmanGazeboPlugin : public KulmanGazeboPlugin
{
 public:
  // Constructor.
  TosmanGazeboPlugin();

  // Destructor.
  virtual ~TosmanGazeboPlugin();

  // Implements Gazebo virtual load function.
  //virtual void Load(physics::ModelPtr model, sdf::ElementPtr /*_sdf*/);

 protected:

  virtual void initSubscribers() override;

  // Inits the ROS publishers.
  virtual void initPublishers() override;

  // Writes simulation state.
  virtual void readSimulation() override;


  // Writes simulation state.
  virtual void writeSimulation() override;

  // Publisher
  ros::Publisher imuDataPublisher_;
  ros::Publisher actuatorDataPublisher_;

};

}
