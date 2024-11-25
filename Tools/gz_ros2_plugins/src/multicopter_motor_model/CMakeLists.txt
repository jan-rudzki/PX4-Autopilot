cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

project(multicopter_motor_model)

# Find the Gazebo Sim 7 package and ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gz-sim7 REQUIRED)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${gz-sim7_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Source files
set(SOURCES
  src/MulticopterMotorModel.cc
)

# Create the plugin library
add_library(MulticopterMotorModel SHARED ${SOURCES})

# Link dependencies
ament_target_dependencies(MulticopterMotorModel
  rclcpp
  std_msgs
  gz-sim7
)

# Install the library
install(TARGETS MulticopterMotorModel
  DESTINATION lib
)

ament_export_dependencies(rclcpp std_msgs gz-sim7)
ament_package()

