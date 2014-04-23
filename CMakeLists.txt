cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

find_package(octomap 1.5 REQUIRED)
include_directories(BEFORE ${OCTOMAP_INCLUDE_DIRS})
link_directories(${OCTOMAP_LIBRARY_DIRS})
link_libraries(${OCTOMAP_LIBRARY_DIRS}/liboctomap.so ${OCTOMAP_LIBRARY_DIRS}/liboctomath.so)
MESSAGE(STATUS "OctoMap paths:" ${OCTOMAP_INCLUDE_DIRS} " " ${OCTOMAP_LIBRARY_DIRS})

rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()

file(GLOB_RECURSE HEADER_FILES include/*.h)
rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

#link to a local version of FCL
#set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)
#find_package(fcl REQUIRED)
message("fcl_INCLUDE_DIRS: "    ${fcl_INCLUDE_DIRS})
message("fcl_LIBRARIES: "       ${fcl_LIBRARIES})
#include_directories(${fcl_INCLUDE_DIRS})
#link_libraries(${fcl_LIBRARIES})

#common commands for building c++ executables and libraries
# OBSOLETE???: src/TreeDescription.cpp
rosbuild_add_library(wholebodycontroller 
                                        src/WholeBodyController.cpp
                                        src/AdmittanceController.cpp
                                        src/ComputeNullspace.cpp                                        
                                        src/Chain.cpp
                                        src/ChainParser.cpp
                                        src/RobotState.cpp
                                        src/Tree.cpp
                                        src/Tracing.cpp

)

rosbuild_add_library(motionobjectives
                                     src/motionobjectives/MotionObjective.cpp
                                     src/motionobjectives/CartesianImpedance.cpp
                                     src/motionobjectives/CollisionAvoidance.cpp
                                     src/motionobjectives/JointLimitAvoidance.cpp
                                     src/motionobjectives/PostureControl.cpp
)
rosbuild_add_library(interfaces
                               src/interfaces/JointTrajectoryAction.cpp
                               src/interfaces/RobotInterface.cpp
)

message("geolib_fcl_INCLUDE_DIRS: "    ${geolib_fcl_INCLUDE_DIRS})
message("geolib_fcl_LIBRARIES: "       ${geolib_fcl_LIBRARIES})
if (geolib_fcl_INCLUDE_DIRS AND geolib_fcl_LIBRARIES)

    rosbuild_add_library(vwm_tools
        src/vwm/vwmclient.cpp
        src/vwm/vwmobject.cpp
    )
    target_link_libraries(wholebodycontroller vwm_tools)
    message(STATUS "WBC uses VWM")
else()
    message(STATUS "WBC does not use VWM")
endif()

#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(whole_body_controller src/whole_body_controller_node.cpp ${HEADER_FILES})
#rosbuild_add_executable(grasp_precompute src/GraspPrecompute.cpp ${HEADER_FILES})
target_link_libraries(whole_body_controller wholebodycontroller codeprofiler)
target_link_libraries(wholebodycontroller motionobjectives)
target_link_libraries(wholebodycontroller interfaces)
