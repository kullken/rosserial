cmake_minimum_required(VERSION 3.10.2)
project(rosserial_mbed)

find_package(catkin REQUIRED COMPONENTS
  message_generation
)

add_message_files(FILES
  Adc.msg
)

add_service_files(FILES
  Test.srv
)

generate_messages()

catkin_package(CATKIN_DEPENDS
  message_runtime
)

install(
  DIRECTORY src/ros_lib
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src
)

catkin_install_python(
  PROGRAMS src/${PROJECT_NAME}/make_libraries.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
