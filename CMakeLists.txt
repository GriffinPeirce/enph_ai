cmake_minimum_required(VERSION 2.8.3)

project(enph_ai)

find_package(catkin REQUIRED rospy std_msgs)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package()