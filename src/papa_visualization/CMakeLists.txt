cmake_minimum_required(VERSION 2.8.3)
project(papa_visualization)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rospy
)
