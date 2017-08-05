cmake_minimum_required(VERSION 2.8.3)
project(rosserial_mbed)

# Disable message generation for lisp and java
set(ENV{ROS_LANG_DISABLE} "genlisp:genjava")

find_package(catkin REQUIRED COMPONENTS
  message_generation
)

add_message_files(FILES
                  Adc.msg
                  wheel_count.msg
                  papa_imu.msg
                  papa_sonar.msg
                 )

add_service_files(FILES
                  Test.srv
                 )

generate_messages()

catkin_package(
  CATKIN_DEPENDS message_runtime
#  CFG_EXTRAS rosserial_mbed-extras.cmake
)

install(DIRECTORY src/ros_lib
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src
)

#install(DIRECTORY mbed-cmake
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

#install(PROGRAMS src/rosserial_mbed/make_libraries.py
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)



