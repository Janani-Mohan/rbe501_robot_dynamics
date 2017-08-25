Project : Kinematic modelling of the ABB IRB 120 for pick-and-place of a BGA chip.
This implementation package was developed for the Hardware implementation.

The location of the chip was identified int he world coordinate frame using a single camera system. Following the BGa chip localization, the arm is commanded to grasp it. After grasping it, the arm moves to a second workspace where a PCB board is placed. The slot where the PCB board must be placed is identified and localized. The Arm is commanded to place the BGA chip at the known location.

Videos:
Hardware implementation : https://youtu.be/u9AgeZEjfQo
Simulation: https://youtu.be/KAda7B-SvCg

Workspace usage:

$ cd dynamics_ws/
$ catkin_make
$ roscore
# open new terminal tab
# this opens moveit!
$ rosrun irb120_moveit moveit_planning_execution sim:=false robot_ip:=<insert_ip_of_abb_arm>
# open new terminal tab
$ rosrun camera_capture camera_capture_node
# open new terminal tab
$ rosrun irb120_tf_calc irb120_tf_calc_node
# open new terminal tab
$ rosrun moveit moveit_node
# open new terminal tab
$ rosrun irb120_perception_pickup irb120_perception_pickup_node
# open new terminal tab
$ rosrun chip_place chip_place_node

