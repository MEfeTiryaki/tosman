cmake_minimum_required(VERSION 2.8.3)
project(tosman_controller_frame)

add_definitions("-std=c++11")

find_package(Boost REQUIRED COMPONENTS thread chrono)

find_package(catkin REQUIRED
  COMPONENTS
  	tosman_model
	  arac_msgs
    arac_joystick
    tosman_controller
    tosman_state_estimator
    roscpp
    param_io
    geometry_msgs
    std_msgs
    sensor_msgs
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)


catkin_package(
  INCLUDE_DIRS
  	include
  LIBRARIES
  	tosman_controller_frame
  CATKIN_DEPENDS
  	tosman_model
  	arac_msgs
    arac_joystick
    tosman_controller
    tosman_state_estimator
  	roscpp
  	param_io
  	geometry_msgs
    std_msgs
    sensor_msgs
  DEPENDS
    Boost
)

###########
## Build ##
###########

include_directories(include)
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})

add_library(
  ${PROJECT_NAME}
  src/TosmanControllerFrame.cpp
)

add_dependencies(tosman_controller_frame
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(tosman_controller_frame
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(tosman_controller_frame_node
  src/tosman_controller_frame_node.cpp
)
add_dependencies(tosman_controller_frame_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(tosman_controller_frame_node
  tosman_controller_frame
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS tosman_controller_frame tosman_controller_frame_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
