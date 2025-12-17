cmake_minimum_required(VERSION 3.0.2)
project(planar_3dof_description)

find_package(catkin REQUIRED COMPONENTS
  joint_state_publisher
  robot_state_publisher
  rviz
  xacro
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)