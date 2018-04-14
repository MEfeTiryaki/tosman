cmake_minimum_required(VERSION 2.8.3)
project(arac)

add_definitions("-std=c++11")

find_package(catkin REQUIRED
  COMPONENTS
  tosman_sim
  tosman_controller
  tosman_controller_frame
  tosman_description
  tosman_gazebo
  tosman_model
  arac_joystick
  arac_msgs
  arac_state_estimator
  arac_rviz
  param_io
)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS
  tosman_sim
  tosman_controller
  tosman_controller_frame
  tosman_description
  tosman_gazebo
  tosman_model
  arac_joystick
  arac_msgs
  arac_state_estimator
  arac_rviz
  param_io
)
