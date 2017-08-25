#ifndef MOVEIT_HW
#define MOVEIT_HW

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometry_msgs/Point.h>

class MoveitNode
{
public:
	ros::Publisher display_publisher;
	ros::Subscriber pick_sub;
	ros::Subscriber place_sub;
	geometry_msgs::Point pose;
	void PickCB(const geometry_msgs::Point pos);
	void PlaceCB(const geometry_msgs::Point pos);
	void PlanTraj_Pickup(); 
	void PlanTraj_Place();
	MoveitNode(ros::NodeHandle n);
};

#endif
