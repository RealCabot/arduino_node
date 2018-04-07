cmake_minimum_required(VERSION 2.8.3)
project(arduino_node)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --Wno-dev" )

find_package(catkin REQUIRED COMPONENTS
  rosserial_arduino
  rosserial_client
  arduino_msg
)

catkin_package()

rosserial_generate_ros_lib(
  PACKAGE rosserial_arduino
  SCRIPT make_libraries.py
)

add_dependencies(arduino_node_ros_lib 
  arduino_msg_generate_messages_cpp
)


rosserial_configure_client(
  DIRECTORY firmware
  TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)

rosserial_add_client_target(firmware oneForAll ALL)
rosserial_add_client_target(firmware oneForAll-upload)
