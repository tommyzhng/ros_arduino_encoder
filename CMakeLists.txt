cmake_minimum_required(VERSION 3.0.2)
project(ros_arduino_encoder)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_arduino_encoder
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(

  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/include/ros_arduino_encoder
  ${CMAKE_SOURCE_DIR}/include/serial/include
)
add_library(ros_arduino_encoder src/ros_arduino_encoder.cpp)
target_include_directories(ros_arduino_encoder PUBLIC include)
target_compile_features(ros_arduino_encoder PUBLIC cxx_std_17)

add_executable(ros_arduino_encoder_node src/ros_arduino_encoder_node.cpp)

target_link_libraries(ros_arduino_encoder_node PUBLIC ros_arduino_encoder ${catkin_LIBRARIES} serial)

add_dependencies(ros_arduino_encoder_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_arduino_encoder
  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

## Include the serial library manually
set(SERIAL_PROJECT_DIR ${CMAKE_SOURCE_DIR}/include/serial)

set(serial_SRCS
  ${SERIAL_PROJECT_DIR}/src/serial.cc
  ${SERIAL_PROJECT_DIR}/src/impl/unix.cc
  ${SERIAL_PROJECT_DIR}/src/impl/list_ports/list_ports_linux.cc
)

add_library(serial STATIC ${serial_SRCS})
target_link_libraries(serial rt pthread)
