cmake_minimum_required(VERSION 2.8.3)

add_compile_options(-std=c++11)

include_directories(
  ${ROS_LIB_DIR}
  include
)

# Note: the name of the library folder should be EXACTLY the same as the .h file
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/Libraries)

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
add_definitions(-DUSB_CON)

generate_arduino_firmware(oneForAll
  SRCS src/main.cpp src/EncoderReader.cpp src/IMUReader.cpp src/PID.cpp src/Motor.cpp src/Touch.cpp ${ROS_LIB_DIR}/time.cpp
  HDRS include/EncoderReader.h include/IMUReader.h include/Motor.h include/Touch.h
  BOARD mega2560
  PORT /dev/ttyACM0
)
