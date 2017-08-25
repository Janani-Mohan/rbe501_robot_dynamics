#ifndef BGAPLACE_H
#define BGAPLACE_H

#include <ros/ros.h>
#include <std_msgs/Int8.h>
//#include <sensor_msgs/ImageConstPtr.h>

#define PI 3.14159265
#define dx 1280 // in pixels
#define dy 720 //in pixels
#define fov_diag 73*PI/180 // degrees
#define height_camera 50 //cm

class BGAPlace{
private:
	ros::Subscriber m_imageSub;
	ros::Subscriber m_goFlag; 
	ros::Publisher m_orientationPub;
	ros::Publisher m_xyzPlacePub;
	ros::Subscriber tf_sub;
	// void flagReceiveCallback(std_msgs::Int8 inFlag);
	void placeProcessCallBack(const sensor_msgs::ImageConstPtr& img);
	void transformCallBack(const geometry_msgs::Point pos);
public:
	BGAPlace(ros::NodeHandle nh);
	geometry_msgs::Point tf;
	
};
#endif
