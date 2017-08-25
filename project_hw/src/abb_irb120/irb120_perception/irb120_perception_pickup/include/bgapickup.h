#ifndef BGAPICKUP_H
#define BGAPICKUP_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>

class BGAPickup
{
public:
	ros::Publisher m_orientationPub;
	ros::Subscriber m_imageSub;
	ros::Subscriber tf_sub;
	ros::Publisher m_xy_pickup_Pub;
	geometry_msgs::Point rwc;
	geometry_msgs::Point tf;
	
	void detectBGACallBack(const sensor_msgs::ImageConstPtr& img); 
	void PickupCB(const geometry_msgs::Point pos); 
	BGAPickup(ros::NodeHandle n);
	
};

#endif
