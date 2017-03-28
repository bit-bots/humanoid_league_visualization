cmake_minimum_required(VERSION 2.8.3)
project(humanoid_league_relative_rqt)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED dynamic_reconfigure std_msgs humanoid_league_msgs rospy)




catkin_python_setup()
## Generate dynamic reconfigure parameters in the 'cfg' folder
 generate_dynamic_reconfigure_options(
   cfg/relative_rqt_params.cfg
)
catkin_package()

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
