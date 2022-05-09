cmake_minimum_required(VERSION 2.8.3)
project(opera_tools)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  roscpp
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ic120_bringup
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
# include
${catkin_INCLUDE_DIRS}
)

add_executable(nav_gazebo_odom src/nav_gazebo_odom.cpp)
target_link_libraries(nav_gazebo_odom
  ${catkin_LIBRARIES}
)
