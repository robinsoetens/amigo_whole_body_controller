cmake_minimum_required(VERSION 2.8.3)
project(amigo_whole_body_controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  message_generation
  actionlib
  actionlib_msgs
  arm_navigation_msgs
  code_profiler
  control_msgs
  sensor_msgs
  geometry_msgs
  kdl_parser
  octomap_msgs
  roscpp
  rospy
  std_msgs
  tf
  visualization_msgs
  fcl
  ed_wbc
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS thread)
find_package(orocos_kdl REQUIRED)
find_package(octomap REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(BULLET bullet)

## Generate messages in the 'msg' folder
add_message_files(FILES
  WholeBodyControllerStatus.msg
)

# add_service_files(FILES
#   # WholeBodyControllerStatus.msg
# )

add_action_files(FILES
  ArmTask.action
)

generate_messages(
  DEPENDENCIES
    std_msgs
    actionlib_msgs
    arm_navigation_msgs
    geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES amigo_whole_body_controller
  CATKIN_DEPENDS
    # TODO, other deps
    std_msgs
    actionlib_msgs
    arm_navigation_msgs
    geometry_msgs
    message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${orocos_kdl_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${BULLET_INCLUDE_DIRS}
)

# link_directories(${catkin_LIBRARY_DIRS})
# link_directories(${Boost_LIBRARY_DIRS})
# link_directories(${orocos_kdl_LIBRARY_DIRS})
# link_directories(${OCTOMAP_LIBRARY_DIRS})

# show the headers in qtcreator
file(GLOB HEADERS
  RELATIVE ${PROJECT_SOURCE_DIR} "include/*.h"
  RELATIVE ${PROJECT_SOURCE_DIR} "include/*/*.h"
  RELATIVE ${PROJECT_SOURCE_DIR} "include/*/*/*.h"
)

# Declare a cpp library
add_library(amigo_whole_body_controller
  ${HEADERS}
  src/WholeBodyController.cpp
  src/AdmittanceController.cpp
  src/ComputeNullspace.cpp
  src/Chain.cpp
  src/ChainParser.cpp
  src/conversions.cpp
  src/ReferenceGenerator.cpp
  src/RobotState.cpp
  src/Tree.cpp
  src/Tracing.cpp

  src/motionobjectives/MotionObjective.cpp
  src/motionobjectives/CartesianImpedance.cpp
  src/motionobjectives/CollisionAvoidance.cpp
  src/motionobjectives/JointLimitAvoidance.cpp
  src/motionobjectives/PostureControl.cpp

  src/interfaces/JointTrajectoryAction.cpp
  src/interfaces/RobotInterface.cpp


# src/vwm/vwmclient.cpp
# src/vwm/vwmobject.cpp
)
target_link_libraries(amigo_whole_body_controller
  ${catkin_LIBRARIES}

  ${orocos_kdl_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${BULLET_LIBRARIES}
)

add_executable(whole_body_controller
  src/whole_body_controller_node.cpp
)
target_link_libraries(whole_body_controller amigo_whole_body_controller)

add_executable(wbc
  src/wbc_node.cpp
)
target_link_libraries(wbc amigo_whole_body_controller)

add_dependencies(amigo_whole_body_controller ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(amigo_whole_body_controller ${catkin_EXPORTED_TARGETS})
