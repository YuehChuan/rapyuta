cmake_minimum_required(VERSION 2.8.3)
project(rapyuta_pose_estimator)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3")

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  cv_bridge
  geometry_msgs
  image_transport
  rapyuta_pose_estimator_lib
  roscpp
  sensor_msgs
  apriltags
  tf
  std_msgs
  rapyuta_msgs
  message_generation
  message_filters
  tf_conversions
)

find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system signals regex date_time program_options thread)

find_package( Sophus REQUIRED )
find_package(Eigen REQUIRED)

# Find GTSAM components
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBRARIES gtsam)   # TODO: automatic search libs

find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
#include(GtsamTesting)

# for unittest scripts
#set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")




# Import the yaml-cpp libraries.
include(FindPkgConfig)
pkg_check_modules(Yaml REQUIRED yaml-cpp)


generate_messages(DEPENDENCIES
    geometry_msgs
    std_msgs
    rapyuta_msgs
)



catkin_package(
INCLUDE_DIRS include
#LIBRARIES rapyuta_pose_estimator
CATKIN_DEPENDS cmake_modules cv_bridge geometry_msgs image_transport rapyuta_pose_estimator_lib roscpp sensor_msgs message_runtime message_filters std_msgs
DEPENDS eigen OpenCV
)

include_directories(
    include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Yaml_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
  )

add_executable(rapyuta_pose_estimator src/node.cpp 	src/rapyuta_pose_estimator.cpp)

add_dependencies(rapyuta_pose_estimator ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

target_link_libraries(
    rapyuta_pose_estimator
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${Yaml_LIBRARIES}
    ${apriltagscpp_LIBRARIES}
    ${Sophus_LIBRARIES}
    ${GTSAM_LIBRARIES}
    )

###nodelet
##add_library(rapyuta_pose_estimator_nodelet src/nodelet.cpp src/rapyuta_pose_estimator.cpp)
##add_dependencies(rapyuta_pose_estimator_nodelet ${catkin_EXPORTED_TARGETS ${PROJECT_NAME}_gencfg)
##
##target_link_libraries(
##    rapyuta_pose_estimator_nodelet
##    ${Yaml_LIBRARIES}
##    ${catkin_LIBRARIES}
##    ${OpenCV_LIBRARIES}
##    )
