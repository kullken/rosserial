cmake_minimum_required(VERSION 3.10.2)
project(rosserial_vex_cortex)

find_package(catkin REQUIRED COMPONENTS)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS)

install(
  DIRECTORY src/ros_lib
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src
)

install(
  PROGRAMS scripts/genproject.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(
  PROGRAMS src/${PROJECT_NAME}/make_libraries.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
