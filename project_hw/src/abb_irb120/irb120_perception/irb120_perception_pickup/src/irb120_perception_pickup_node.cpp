#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include "bgapickup.h"
#include <math.h> 
#include <string.h>
#include <std_msgs/Float64.h>
#include <iostream>
#define PI 3.14159265
#define diag_fov 73// in  degrees
#define heightCam 50 //in cm

using namespace std;

BGAPickup::BGAPickup(ros::NodeHandle n){
	m_orientationPub =  n.advertise<std_msgs:: Float64> ("/detect/bga/orientation",100);
	m_xy_pickup_Pub = n.advertise<geometry_msgs:: Point> ("/detect/bga_pickup/xy",100);
	
	tf_sub = n.subscribe("irb120/transform", 2, &BGAPickup::PickupCB,this);
	m_imageSub = n.subscribe("camera/image", 5, &BGAPickup::detectBGACallBack,this);
}
void BGAPickup::PickupCB(const geometry_msgs::Point pos)
{

   tf.x = pos.x;
   tf.y = pos.y; 
}

void BGAPickup::detectBGACallBack(const sensor_msgs::ImageConstPtr& img)
{
	static int flag = 0;
	int dx = 1280, dy = 720;
	cv::Mat bgaOriginal, Bga_Chip_gray,Bga_Chip_thresh,imageFilter,imgBF,imageRGB;
	cv::vector<cv::Point2f> coordsInCm;
	cv_bridge::CvImagePtr inMsgPtr;
	inMsgPtr = cv_bridge::toCvCopy(img,sensor_msgs::image_encodings::BGR8);

	bgaOriginal = inMsgPtr->image;
	int px = bgaOriginal.cols, py = bgaOriginal.rows;
	float diagInPix = abs(sqrt(dx*dx+dy*dy));
	float pixPerUnit = 1/((heightCam*tan(diag_fov*.5*PI/180))/(diagInPix*.5));

	cv::cvtColor(bgaOriginal, Bga_Chip_gray, CV_BGR2GRAY);
	cv::threshold(Bga_Chip_gray,Bga_Chip_thresh, 127.0, 255.0, cv::THRESH_BINARY);
	cv:: bilateralFilter(Bga_Chip_thresh,imageFilter,9,75,75);

	/*drawing contours and finding object centre and image centre */
	cv::vector<cv::vector<cv::Point> > contours;
	cv::vector<cv::Vec4i> hierarchy	;
	cv::Mat imageCopy  = imageFilter.clone();
	
	cv::findContours(imageCopy,contours,hierarchy, cv::RETR_CCOMP ,cv::CHAIN_APPROX_NONE,cv::Point(0,0));
	cv::vector<cv::vector <cv::Point> > contours_poly(contours.size());
	cv::vector<cv::Rect> boundRect(contours.size());
	cv::Mat drawing = cv::Mat::zeros( bgaOriginal.size(), CV_8UC3 );
	cv::vector<int> width, height;
	cv::vector<cv::Point2f> points;
	int xc, yc;
	cv::vector<cv::RotatedRect> minRect( contours.size() );
	int itr = 0;
	geometry_msgs::Point realWorldCoords;
	geometry_msgs::Point rwc;
	std_msgs::Float64 orientation;
	rwc.x = 0;
	rwc.y = 0;
	realWorldCoords.x = 0;
	realWorldCoords.y=0;
	int counter=0;
	for(unsigned int i = 0; i< contours.size(); i++)
	{
		double area =cv::contourArea(contours[i], true);
		cv::Moments moment = cv::moments((cv::Mat)contours[i]);
		if (std::abs(area)>400 and std::abs(area)<1500)
		{
			cv::approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 3, true );
			boundRect[i] = boundingRect(contours_poly[i]);
			float w = boundRect[i].width, h = boundRect[i].height;  
			
 			if ( w/h>.90 and h/w<1.09 and h>20.0 and h<40.0 and w>20.0 and w<40.0)
			{
			points.push_back(cv::Point2f(moment.m10 / moment.m00, moment.m01 / moment.m00) );
			cv::circle(bgaOriginal, points.back(), 3, cv::Scalar(0,0,255), 2, 8, 0);
			cv::rectangle( bgaOriginal, boundRect[i].tl(), boundRect[i].br(), cv::Scalar(255,0,0), 3, 8, 0 );
			cv::drawContours( bgaOriginal, contours, i, cv::Scalar(0,255,0), 2, 8, hierarchy, 0, cv::Point() );
			minRect[i] = cv::minAreaRect(contours[i]);
			float angle = minRect[i].angle;
			std::cout<<angle<<" deg"<<std::endl;
			orientation.data = angle;
			rwc.x = ((points[itr].x-px*.5)/pixPerUnit);
			rwc.y = ((py*.5-points[itr].y)/pixPerUnit);
			rwc.z = heightCam;
			flag = flag + 1;
			// cout<<"Contour centre pts "<<points[itr]<<endl;
			// cout<<"Real world coord "<<rwc<<endl;
			// cout<<"pix per unit "<<pixPerUnit<<endl;
			// cout<<"height is "<<heightCam*pixPerUnit<<endl;
			
			counter+=1;
			cout<<"contour size "<<counter<<endl;
			}
			if(flag == 40)
			{  cout<<"Contour centre pts "<<points[itr]<<endl;
			   realWorldCoords.x = tf.x + (rwc.x/100);
			   realWorldCoords.y = tf.y + (rwc.y/100);
			   cout<<"Real world coords "<<realWorldCoords<<endl;
			   m_xy_pickup_Pub.publish(realWorldCoords);
			   flag = 41;
			   break;
			}
		 }
	}
	// points.push_back(cv::Point2f(px*.5,py*.5));
	// cv::circle(bgaOriginal,points.back(), 5, cv::Scalar(0,0,0), 3, 8, 0);
	cout<<flag<<endl;
	cv::imshow("contours", bgaOriginal);
	cv::waitKey(1);
}
int main(int argc, char**argv)
{
	ros::init(argc, argv, "irb120_perception_pickup_node");
	ros::NodeHandle n;
	BGAPickup detect(n);
	ros::spin();
}
