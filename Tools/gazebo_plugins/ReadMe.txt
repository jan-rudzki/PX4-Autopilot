################################################################################################################################################
Advanced Lift Drag Plugin:
- can be modified with gzdbg statements like this "gzdbg << "Speed in Lift-Drag Plane: " << speedInLDPlane << "\n";" for debugging
- after modification go to directory and execute "make" --> this creates an updated version of the target .so in the build directory
- if problems occur remove the build directory and type "cmake .." and "make" in the command line
- in the .sdf file the plugin target is specified like this (    <plugin filename="/home/jan/Desktop/PX4/PX4-Autopilot/Tools/ros2_ws/install/ advanced_lift_drag_plugin/lib/libAdvancedLiftDrag.so" name="gz::sim::systems::AdvancedLiftDrag">
). by default it points to the version with the ROS2 node that publishes data. change it to the .so of the gazebo_plugins directory and you get the plugin without the ros2 extension
- to see the debug statements set verbosity to 4 in px4-rc.simulator in ROMFS/px4fmu_common/init.d-posix
################################################################################################################################################
Multicopter Motor Model Plugin: tbd
