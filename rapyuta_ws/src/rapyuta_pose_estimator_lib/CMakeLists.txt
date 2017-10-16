cmake_minimum_required(VERSION 2.8.3)
project(rapyuta_pose_estimator_lib)
set(CMAKE_CXX_FLAGS "-O3")

find_package(catkin REQUIRED COMPONENTS 
    cmake_modules
    cv_bridge
    geometry_msgs
    image_transport
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
find_package(Boost REQUIRED COMPONENTS system signals regex date_time program_options thread)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)

# Find GTSAM components
find_package(GTSAM REQUIRED) # Uses installed package
#include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBRARIES gtsam)   # TODO: automatic search libs

find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)

# for unittest scripts
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")

generate_messages(DEPENDENCIES
    geometry_msgs
    std_msgs
    rapyuta_msgs
)


catkin_package(
  INCLUDE_DIRS include
  DEPENDS eigen opencv
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/rapyuta_lib.cpp
# )

file(GLOB SOURCEFILES "src/*.cpp")
add_library(${PROJECT_NAME} SHARED ${SOURCEFILES})

add_dependencies(${PROJECT_NAME} SHARED ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS} ${GTSAM_LIBRARIES})

