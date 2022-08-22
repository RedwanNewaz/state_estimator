cmake_minimum_required(VERSION 2.8.3)
project(state_estimator)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES state_estimator
#  CATKIN_DEPENDS cv_bridge geometry_msgs roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/state_estimator.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
set(SOURCE_FILES
        src/EKF.cpp
        src/FilterBase.cpp
        src/PerceptionInterface.cpp include/state_estimator/PerceptionInterface.h)

set(HEADER_FILES
        include/state_estimator/EKF.h
        include/state_estimator/robot_defs.h
        include/state_estimator/FilterBase.h
        src/PerceptionInterface.cpp include/state_estimator/PerceptionInterface.h)

include_directories(include/state_estimator)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
 add_executable(moving_camera_localization src/moving_camera_localization.cpp ${SOURCE_FILES} ${HEADER_FILES})

## Specify libraries to link a library or executable target against
target_link_libraries(moving_camera_localization
        ${catkin_LIBRARIES}
        Eigen3::Eigen
        )