cmake_minimum_required(VERSION 2.8.3)
project(rapyuta_pose_estimator_lib)
set(CMAKE_CXX_FLAGS "-O3")

find_package(catkin REQUIRED COMPONENTS cmake_modules)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)

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
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/rapyuta_lib.cpp
# )

file(GLOB SOURCEFILES "src/*.cpp")
add_library(${PROJECT_NAME} SHARED ${SOURCEFILES})

add_dependencies(${PROJECT_NAME} SHARED ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS})

