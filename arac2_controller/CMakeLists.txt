cmake_minimum_required(VERSION 2.8.3)
project(arac2_controller)

add_definitions("-std=c++11")

find_package(catkin REQUIRED
  COMPONENTS
  arac_model
  arac_controller
  param_io
)

catkin_package(
  INCLUDE_DIRS
  	include
  LIBRARIES
    arac2_controller
  CATKIN_DEPENDS
	  arac_model
    arac_controller
    param_io
)


# if  ${catkin_INCLUDE_DIRS} is not added, c code in can't find headers from other
# packages
# if  inclde is not added, c code in can't find headers from this package can't be
# found
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(
  ${PROJECT_NAME}
  src/Arac2OLController.cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
