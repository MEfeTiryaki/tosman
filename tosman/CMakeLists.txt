cmake_minimum_required(VERSION 2.8.3)
project(tosman)

add_definitions("-std=c++11")

find_package(catkin REQUIRED
  COMPONENTS
  tosman_sim
  tosman_controller
  tosman_controller_frame
  tosman_description
  tosman_gazebo
  tosman_model
  tosman_state_estimator
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
  tosman_state_estimator
)
