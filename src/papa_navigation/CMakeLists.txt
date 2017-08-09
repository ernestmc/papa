cmake_minimum_required(VERSION 2.8.3)
project(papa_navigation)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES arlobot_description
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

## Installation rules

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
