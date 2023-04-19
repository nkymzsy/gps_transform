cmake_minimum_required(VERSION 3.0.2)
project(gps_transform)
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Relase")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})


include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL
  CATKIN_DEPENDS
  std_msgs
  nav_msgs
)


add_executable(gps_trans src/gps_trans.cpp)
target_link_libraries(gps_trans ${catkin_LIBRARIES} ${PCL_LIBRARIES} )
