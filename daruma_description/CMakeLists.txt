cmake_minimum_required(VERSION 3.0.2)
project(daruma_description)

find_package(catkin REQUIRED COMPONENTS
  rviz
)

catkin_python_setup()


catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS ${python_scripts_DIR}
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})