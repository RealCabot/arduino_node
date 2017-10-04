cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})

# Note: the name of the library folder should be EXACTLY the same as the .h file
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/Libraries)

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
add_definitions(-DUSB_CON)

generate_arduino_firmware(arduinoEncoder
  SRCS readEncoders.cpp ${ROS_LIB_DIR}/time.cpp
  BOARD mega2560
  PORT /dev/ttyACM0
)
