cmake_minimum_required(VERSION 2.8.3)
project(tosman_controller)

add_definitions("-std=c++11")

find_package(catkin REQUIRED
  COMPONENTS
  kulman_controller
  tosman_model
  param_io
)

catkin_package(
  INCLUDE_DIRS
  	include
  LIBRARIES
    tosman_controller
  CATKIN_DEPENDS
    kulman_controller
	  tosman_model
    param_io
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(
  ${PROJECT_NAME}
  src/TosmanOLController.cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
