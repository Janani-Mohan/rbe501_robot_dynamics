//Move It header files
#include<moveit_hw.h>

static int flag = 0;
MoveitNode::MoveitNode(ros::NodeHandle n)
{
	display_publisher = n.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    pick_sub = n.subscribe("/detect/bga_pickup/xy",5,&MoveitNode::PickCB,this);
    place_sub = n.subscribe("/detect/bga_place/xy",10,&MoveitNode::PlaceCB,this);
} 

void MoveitNode::PickCB(const geometry_msgs::Point pos)
{
 
	pose.x = pos.x ;
	pose.y = pos.y ;
	ROS_INFO("x%f",pose.x);
	ROS_INFO("y%f",pose.y);
	
	PlanTraj_Pickup();

}

void MoveitNode::PlaceCB(const geometry_msgs::Point pos)
{
 
	pose.x = pos.x;
	pose.y = pos.y;
	ROS_INFO("x%f",pose.x);
	ROS_INFO("y%f",pose.y);
	PlanTraj_Place();

}

void MoveitNode::PlanTraj_Pickup()
{

	moveit::planning_interface::MoveGroup group("manipulator");
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	moveit_msgs::DisplayTrajectory display_trajectory;
		// Trajectory-------2
	robot_state::RobotState start_state2(*group.getCurrentState());
	geometry_msgs::Pose start_pose2;
	start_pose2.orientation.w = 0.5;
	start_pose2.orientation.x = -0.5;
	start_pose2.orientation.y = 0.5;
	start_pose2.orientation.z = 0.5;
	start_pose2.position.x = 0.200;
	start_pose2.position.y = 0.350;
	start_pose2.position.z = 1.159;
	const robot_state::JointModelGroup *joint_model_group2 = start_state2.getJointModelGroup(group.getName());
	start_state2.setFromIK(joint_model_group2, start_pose2);
	group.setStartState(start_state2);

	geometry_msgs::Pose target_pose2;
	target_pose2.orientation.w = 0.5;
	target_pose2.orientation.x= -0.5;
	target_pose2.orientation.y = 0.5;
	target_pose2.orientation.z = 0.5;
	target_pose2.position.x = pose.x;
	target_pose2.position.y = pose.y;
	target_pose2.position.z = 0.770;
	group.setPoseTarget(target_pose2);
	///Motion plan from current location to custom position
	moveit::planning_interface::MoveGroup::Plan my_plan2;
	bool success2 = group.plan(my_plan2);
	ROS_INFO("Visualizing plan 2 (pose goal)%s",success2?"":"FAILED");
	 // Sleep to give RViz time to visualize the plan. 
	group.move();
	sleep(5.0);

	// // Trajectory-------5
	// robot_state::RobotState start_state5(*group.getCurrentState());
	// geometry_msgs::Pose start_pose5;
	// start_pose5.orientation.w = 0.5;
	// start_pose5.orientation.x = -0.5;
	// start_pose5.orientation.y = 0.5;
	// start_pose5.orientation.z = 0.5;
	// start_pose5.position.x = pose.x;
	// start_pose5.position.y = pose.y;
	// start_pose5.position.z = 1.159;
	// const robot_state::JointModelGroup *joint_model_group5 = start_state5.getJointModelGroup(group.getName());
	// start_state5.setFromIK(joint_model_group5, start_pose5);
	// group.setStartState(start_state5);

	// geometry_msgs::Pose target_pose5;
	// target_pose5.orientation.w = 0.5;
	// target_pose5.orientation.x= -0.5;
	// target_pose5.orientation.y = 0.5;
	// target_pose5.orientation.z = 0.5;
	// target_pose5.position.x = pose.x;
	// target_pose5.position.y = pose.y;
	// target_pose5.position.z = 0.770;
	// group.setPoseTarget(target_pose5);
	// ///Motion plan from current location to custom position
	// moveit::planning_interface::MoveGroup::Plan my_plan5;
	// bool success5 = group.plan(my_plan5);
	// ROS_INFO("Visualizing plan 3 (pose goal)%s",success5?"":"FAILED");
	//  // Sleep to give RViz time to visualize the plan. 
	// group.move();
	// sleep(5.0);


	// Trajectory-------3
	robot_state::RobotState start_state3(*group.getCurrentState());
	geometry_msgs::Pose start_pose3;
	start_pose3.orientation.w = 0.5;
	start_pose3.orientation.x = -0.5;
	start_pose3.orientation.y = 0.5;
	start_pose3.orientation.z = 0.5;
	start_pose3.position.x = pose.x;
	start_pose3.position.y = pose.y;
	start_pose3.position.z = 0.770;
	const robot_state::JointModelGroup *joint_model_group3 = start_state3.getJointModelGroup(group.getName());
	start_state3.setFromIK(joint_model_group3, start_pose3);
	group.setStartState(start_state3);

	geometry_msgs::Pose target_pose3;
	target_pose3.orientation.w = 0.5;
	target_pose3.orientation.x= -0.5;
	target_pose3.orientation.y = 0.5;
	target_pose3.orientation.z = 0.5;
	target_pose3.position.x = 0.200;
	target_pose3.position.y = 0.350;
	target_pose3.position.z = 1.159;
	group.setPoseTarget(target_pose3);
	///Motion plan from current location to custom position
	moveit::planning_interface::MoveGroup::Plan my_plan3;
	bool success3 = group.plan(my_plan3);
	ROS_INFO("Visualizing plan 4 (pose goal)%s",success3?"":"FAILED");
	/* Sleep to give RViz time to visualize the plan. */
	group.move();
	sleep(5.0);



	// Trajectory-------4
	robot_state::RobotState start_state4(*group.getCurrentState());
	geometry_msgs::Pose start_pose4;
	start_pose4.orientation.w = 0.5;
	start_pose4.orientation.x = -0.5;
	start_pose4.orientation.y = 0.5;
	start_pose4.orientation.z = 0.5;
	start_pose4.position.x = 0.200;
	start_pose4.position.y = 0.350;
	start_pose4.position.z = 1.590;
	const robot_state::JointModelGroup *joint_model_group4 = start_state4.getJointModelGroup(group.getName());
	start_state4.setFromIK(joint_model_group4, start_pose4);
	group.setStartState(start_state4);

	geometry_msgs::Pose target_pose4;
	target_pose4.orientation.w = 0.5;
	target_pose4.orientation.x= -0.5;
	target_pose4.orientation.y = 0.5;
	target_pose4.orientation.z = 0.5;
	target_pose4.position.x = -0.200;
	target_pose4.position.y = 0.350;
	target_pose4.position.z = 1.159;
	group.setPoseTarget(target_pose4);
	///Motion plan from current location to custom position
	moveit::planning_interface::MoveGroup::Plan my_plan4;
	bool success4 = group.plan(my_plan4);
	ROS_INFO("Visualizing plan 5 (pose goal)%s",success4?"":"FAILED");
	/* Sleep to give RViz time to visualize the plan. */
	group.move();
	sleep(5.0);
	
}

void MoveitNode::PlanTraj_Place()
{

	moveit::planning_interface::MoveGroup group("manipulator");
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	moveit_msgs::DisplayTrajectory display_trajectory;
		// Trajectory-------5
	robot_state::RobotState start_state5(*group.getCurrentState());
	geometry_msgs::Pose start_pose5;
	start_pose5.orientation.w = 0.5;
	start_pose5.orientation.x = -0.5;
	start_pose5.orientation.y = 0.5;
	start_pose5.orientation.z = 0.5;
	start_pose5.position.x = -0.200;
	start_pose5.position.y = 0.350;
	start_pose5.position.z = 1.159;
	const robot_state::JointModelGroup *joint_model_group5 = start_state5.getJointModelGroup(group.getName());
	start_state5.setFromIK(joint_model_group5, start_pose5);
	group.setStartState(start_state5);

	geometry_msgs::Pose target_pose5;
	target_pose5.orientation.w = 0.5;
	target_pose5.orientation.x= -0.5;
	target_pose5.orientation.y = 0.5;
	target_pose5.orientation.z = 0.5;
	target_pose5.position.x = pose.x;
	target_pose5.position.y = pose.y;
	target_pose5.position.z = 1.159;
	group.setPoseTarget(target_pose5);
	///Motion plan from current location to custom position
	moveit::planning_interface::MoveGroup::Plan my_plan5;
	bool success5 = group.plan(my_plan5);
	ROS_INFO("Visualizing plan 5 (pose goal)%s",success5?"":"FAILED");
	/* Sleep to give RViz time to visualize the plan. */
	group.move();
	sleep(5.0);

	//Trajectory 6
	robot_state::RobotState start_state6(*group.getCurrentState());
	geometry_msgs::Pose start_pose6;
	start_pose6.orientation.w = 0.5;
	start_pose6.orientation.x = -0.5;
	start_pose6.orientation.y = 0.5;
	start_pose6.orientation.z = 0.5;
	start_pose6.position.x = pose.x;
	start_pose6.position.y = pose.y;
	start_pose6.position.z = 1.159;
	const robot_state::JointModelGroup *joint_model_group6 = start_state5.getJointModelGroup(group.getName());
	start_state6.setFromIK(joint_model_group6, start_pose6);
	group.setStartState(start_state6);

	geometry_msgs::Pose target_pose6;
	target_pose6.orientation.w = 0.5;
	target_pose6.orientation.x= -0.5;
	target_pose6.orientation.y = 0.5;
	target_pose6.orientation.z = 0.5;
	target_pose6.position.x = pose.x;
	target_pose6.position.y = pose.y;
	target_pose6.position.z = 0.770;
	group.setPoseTarget(target_pose6);
	///Motion plan from current location to custom position
	moveit::planning_interface::MoveGroup::Plan my_plan6;
	bool success6 = group.plan(my_plan6);
	ROS_INFO("Visualizing plan 6 (pose goal)%s",success6?"":"FAILED");
	/* Sleep to give RViz time to visualize the plan. */
	group.move();
	sleep(5.0);

	// Trajectory-------7
	robot_state::RobotState start_state7(*group.getCurrentState());
	geometry_msgs::Pose start_pose7;
	start_pose7.orientation.w = 0.5;
	start_pose7.orientation.x = -0.5;
	start_pose7.orientation.y = 0.5;
	start_pose7.orientation.z = 0.5;
	start_pose7.position.x = pose.x;
	start_pose7.position.y = pose.y;
	start_pose7.position.z = 0.770;
	const robot_state::JointModelGroup *joint_model_group7 = start_state7.getJointModelGroup(group.getName());
	start_state7.setFromIK(joint_model_group7, start_pose7);
	group.setStartState(start_state7);

	geometry_msgs::Pose target_pose7;
	target_pose7.orientation.w = 0.5;
	target_pose7.orientation.x= -0.5;
	target_pose7.orientation.y = 0.5;
	target_pose7.orientation.z = 0.5;
	target_pose7.position.x = -0.040;
	target_pose7.position.y = 0.302;
	target_pose7.position.z = 1.159;
	group.setPoseTarget(target_pose7);
	///Motion plan from current location to custom position
	moveit::planning_interface::MoveGroup::Plan my_plan7;
	bool success7 = group.plan(my_plan7);
	ROS_INFO("Visualizing plan 7 (pose goal)%s",success7?"":"FAILED");
	/* Sleep to give RViz time to visualize the plan. */
	group.move();
	sleep(5.0);	
	ros::shutdown();
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_custom_node");
	ros::NodeHandle node_handle;
	
	MoveitNode mv(node_handle);
	ros::AsyncSpinner spinner(1);
	spinner.start();

	moveit::planning_interface::MoveGroup group("manipulator");
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	moveit_msgs::DisplayTrajectory display_trajectory;
	
	

	///Setting custom goal position

	// Trajectory------1
	robot_state::RobotState start_state0(*group.getCurrentState());
	geometry_msgs::Pose start_pose0;
	start_pose0.orientation.w = 0.50;
	start_pose0.orientation.x = -0.5;
	start_pose0.orientation.y = 0.5;
	start_pose0.orientation.z = 0.5;
	start_pose0.position.x = -0.040;
	start_pose0.position.y = 0.302;
	start_pose0.position.z = 1.159;
	const robot_state::JointModelGroup *joint_model_group0 = start_state0.getJointModelGroup(group.getName());
	start_state0.setFromIK(joint_model_group0, start_pose0);
	group.setStartState(start_state0);

	geometry_msgs::Pose target_pose0;
	target_pose0.orientation.w = 0.50;
	target_pose0.orientation.x= -0.50;
	target_pose0.orientation.y = 0.50;
	target_pose0.orientation.z = 0.50;
	target_pose0.position.x = 0.2;
	target_pose0.position.y = 0.350;
	// target_pose1.position.x = 0.200;
	// target_pose1.position.y = 0.350;
	target_pose0.position.z = 1.159;
	group.setPoseTarget(target_pose0);
	///Motion plan from current location to custom position
	moveit::planning_interface::MoveGroup::Plan my_plan0;
	bool success0 = group.plan(my_plan0);
	ROS_INFO("Visualizing plan 1 (pose goal)%s",success0?"":"FAILED");
	/* Sleep to give RVizros::shutdown(); time to visualize the plan. */
	group.move();
	sleep(5.0);
	ros::spin();
	ros::shutdown();
	return 0;
}