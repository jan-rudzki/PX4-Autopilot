cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

project(AdvancedLiftDragPlugin)

# Include GNUInstallDirs to define standard installation directories
include(GNUInstallDirs)

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
  src/AdvancedLiftDrag.cc
)

# Create the plugin library
add_library(AdvancedLiftDrag SHARED ${SOURCES})

# Include directories
target_include_directories(AdvancedLiftDrag
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    )

# Link against Gazebo Sim libraries
target_link_libraries(AdvancedLiftDrag gz-sim7::gz-sim7)

# Install the plugin library
install(TARGETS AdvancedLiftDrag
  DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

