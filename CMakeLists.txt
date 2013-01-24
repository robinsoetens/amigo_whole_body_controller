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
set(ROS_BUILD_TYPE Release)

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

#common commands for building c++ executables and libraries
# OBSOLETE???: src/TreeDescription.cpp
rosbuild_add_library(wholebodycontroller 
					src/WholeBodyController.cpp
					src/CartesianImpedance.cpp
					src/AdmittanceController.cpp
					src/ComputeNullspace.cpp
					src/ObstacleAvoidance.cpp
                                        src/JointLimitAvoidance.cpp
                                        src/PostureControl.cpp
                                        src/Component.cpp
                                        src/Chain.cpp
                                        src/ChainParser.cpp
                                        src/MotionObjective.cpp
					src/RobotState.cpp
)
#                                        src/JointTrajectoryAction.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(whole_body_controller src/whole_body_controller_node.cpp ${HEADER_FILES})
#rosbuild_add_executable(grasp_precompute src/GraspPrecompute.cpp ${HEADER_FILES})
target_link_libraries(whole_body_controller wholebodycontroller)
