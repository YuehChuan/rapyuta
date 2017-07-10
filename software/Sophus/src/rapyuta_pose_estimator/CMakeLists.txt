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
)

find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)

# Import the yaml-cpp libraries.
include(FindPkgConfig)
pkg_check_modules(Yaml REQUIRED yaml-cpp)



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
  )

add_executable(rapyuta_pose_estimator src/node.cpp src/rapyuta_pose_estimator.cpp)

add_dependencies(rapyuta_pose_estimator ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

target_link_libraries(
    rapyuta_pose_estimator
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${Yaml_LIBRARIES}
    ${apriltagscpp_LIBRARIES}
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
