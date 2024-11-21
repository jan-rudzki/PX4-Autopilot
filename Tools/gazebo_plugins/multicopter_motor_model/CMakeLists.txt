cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

project(MulticopterMotorModel)

# Find the Gazebo Sim 7 package
find_package(gz-sim7 REQUIRED)

# Set C++ standard
set (CMAKE_CXX_STANDARD 17)

# Include Gazebo Sim headers
include_directories(
  ${gz-sim7_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
)

# Plugin source files
set(SOURCES
  src/MulticopterMotorModel.cc
)

# Create the plugin library
add_library(MulticopterMotorModel SHARED ${SOURCES})

# Link against Gazebo Sim libraries
target_link_libraries(MulticopterMotorModel gz-sim7::gz-sim7)

