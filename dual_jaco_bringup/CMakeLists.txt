cmake_minimum_required(VERSION 3.0.2)
project(dual_jaco_bringup)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  dual_jaco_description
  kinova_driver
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dual_jaco_bringup
#  CATKIN_DEPENDS dual_jaco_description kinova_driver
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
