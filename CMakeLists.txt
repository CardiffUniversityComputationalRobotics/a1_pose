cmake_minimum_required(VERSION 3.0.2)
project(a1_pose)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES a1_pose
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
