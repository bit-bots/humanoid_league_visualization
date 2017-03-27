cmake_minimum_required(VERSION 2.8.3)
project(humanoid_league_image_marker)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  humanoid_league_msgs
  geometry_msgs
  dynamic_reconfigure
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
   cfg/image_marker_params.cfg
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
