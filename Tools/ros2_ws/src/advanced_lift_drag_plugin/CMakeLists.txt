cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

project(advanced_lift_drag_plugin)

# Find the Gazebo Sim 7 package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gz-sim7 REQUIRED)


# Set C++ standard
set (CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Include Gazebo Sim headers
# include_directories(
# 	${rclcpp_INCLUDE_DIRS}
# 	${std_msgs_INCLUDE_DIRS}
# 	${gz-sim7_INCLUDE_DIRS}
# 	${CMAKE_CURRENT_SOURCE_DIR}/src
# )



# Create the plugin library
add_library(AdvancedLiftDrag SHARED src/AdvancedLiftDrag.cc)

# Link dependencies
ament_target_dependencies(AdvancedLiftDrag
  rclcpp
  std_msgs
  gz-sim7
)

install(TARGETS AdvancedLiftDrag DESTINATION lib)

ament_export_dependencies(rclcpp std_msgs gz-sim7)
ament_package()


