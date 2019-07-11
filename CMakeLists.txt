cmake_minimum_required(VERSION 2.8.3)

project(enph_ai)

find_package(catkin REQUIRED rospy std_msgs genmsg actionlib_msgs geometry_msgs)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_action_files(
  DIRECTORY action
  FILES GoalPose.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs geometry_msgs
)

catkin_package(
	CATKIN_DEPENDS actionlib_msgs
)