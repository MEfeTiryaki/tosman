cmake_minimum_required(VERSION 2.8.3)
project(tosman_sim)

find_package(catkin REQUIRED
  COMPONENTS
    tosman_description
    tosman_gazebo
    tosman_controller_frame
    tosman_controller
)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS
    tosman_description
    tosman_gazebo
    tosman_controller_frame
    tosman_controller  
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)
