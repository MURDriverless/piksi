cmake_minimum_required(VERSION 3.0.2)
project(piksi)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})



add_executable(piksi
    src/main.cpp
    src/common.h
    src/sbp.h
    src/navigation.h
    src/imu.h 
)

target_link_libraries(piksi libsbp.a ${catkin_LIBRARIES})