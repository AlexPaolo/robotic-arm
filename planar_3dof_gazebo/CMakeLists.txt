cmake_minimum_required(VERSION 3.0.2)
project(planar_3dof_gazebo)


find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  gazebo_ros_control
  planar_3dof_description
  xacro
)

catkin_package(
)

include_directories(

  ${catkin_INCLUDE_DIRS}
)