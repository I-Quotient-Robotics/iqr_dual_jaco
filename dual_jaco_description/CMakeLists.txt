cmake_minimum_required(VERSION 3.0.2)
project(dual_jaco_description)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  kinova_description
  pan_tilt_description
  urdf
  xacro
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dual_jaco_description
#  CATKIN_DEPENDS kinova_description pan_tilt_description urdf xacro
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
