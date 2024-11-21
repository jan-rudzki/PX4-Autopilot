################################################################################################################################################
Advanced Lift Drag Plugin:
- can be extended with other publishers that provide data of the calculations in the advanced lift drag plugin
- after modification use the colcon build function to recompile the plugin:
	rm -rf build install log	
	colcon build --packages-select advanced_lift_drag_plugin
- to use, specify the plugin target in .sdf file like this:     <plugin filename="/home/jan/Desktop/PX4/PX4-Autopilot/Tools/ros2_ws/install/advanced_lift_drag_plugin/lib/libAdvancedLiftDrag.so" name="gz::sim::systems::AdvancedLiftDrag">

################################################################################################################################################
