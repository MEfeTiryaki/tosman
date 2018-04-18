cmake_minimum_required(VERSION 2.8.3)
project(tosman_state_estimator)

add_definitions("-std=c++11")

find_package(catkin REQUIRED
  COMPONENTS
  kulman_state_estimator
  arac_state_estimator
  kulman_msgs
  tosman_model
  arac_model
  roscpp
  param_io
  geometry_msgs
  std_msgs
  sensor_msgs


)


find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

catkin_package(
  INCLUDE_DIRS
    include
    ${catkin_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    kulman_state_estimator
    tosman_model
    kulman_msgs
 	  arac_model
    roscpp
    param_io
    geometry_msgs
    std_msgs
    sensor_msgs
  DEPENDS
)



add_library(
  ${PROJECT_NAME}
  src/TosmanEKF.cpp
  src/TosmanPerfectEstimator.cpp
)

add_dependencies(
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


add_dependencies(
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)


#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES Readme.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
