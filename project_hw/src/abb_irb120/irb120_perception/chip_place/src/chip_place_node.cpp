#include <ros/ros.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/core.hpp>
#include <iostream>
#include <math.h> /* tan */
#include <opencv2/opencv.hpp>
#include <std_msgs/Int8.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include "chip_place_node.h" 

using namespace cv;
using namespace std;

// std_msgs::Int8 flag;
// flag.data = 0;

BGAPlace::BGAPlace(ros::NodeHandle nh){
	m_orientationPub =  nh.advertise<std_msgs:: Float64> ("/detect/bga/orientation",100);
	m_xyzPlacePub = nh.advertise<geometry_msgs:: Point> ("/detect/bga_place/xy",100);
	tf_sub = nh.subscribe("irb120/transform", 5, &BGAPlace::transformCallBack,this);
	m_imageSub = nh.subscribe("/camera/image", 5, &BGAPlace::placeProcessCallBack, this);
	// m_xyzPlacePub = nh.advertise<geometry_msgs:: Point> ("/camera/place/xyz", 100);
	// m_goFlag = nh.subscribe("camera/process_flag", 5,&BGAPlace::flagReceiveCallback, this);
}
// void BGAPlace::flagReceiveCallback( std_msgs::Int8 inFlag){
// 	flag.data = inFlag.data;
// 	std::cout<<"flag "<<inFlag<<std::endl;
// }

void BGAPlace::transformCallBack(const geometry_msgs::Point pos)
{

   tf.x = pos.x;
   tf.y = pos.y; 
}
void BGAPlace::placeProcessCallBack(const sensor_msgs::ImageConstPtr& img){
	static int flag = 0;
	cv::Mat inImage;
	cv::Mat addWt1,addWt11, imageLab_b, imageFilter, imageFilterN,imageFilter2, imageFilter3,imgBF, imageAdaptiveT,imageRGB,imageLab,imageLab_a;

	cv::Scalar mean[3], standardDeviation[3];
	cv_bridge::CvImagePtr inMsgPtr;

	inMsgPtr = cv_bridge::toCvCopy(img,sensor_msgs::image_encodings::BGR8);
	inImage = inMsgPtr->image;
	
	std::vector<Mat> lab, lab_a, lab_b;

	int px = inImage.cols, py = inImage.rows;
	float diagInPix = abs(sqrt(dx*dx+dy*dy));
	float pixPerUnit = 1/((height_camera*tan(fov_diag*.5))/(diagInPix*.5));

	cv::bilateralFilter(inImage, imgBF, 10,100,100);
	cv::cvtColor(imgBF, imageRGB, COLOR_BGR2RGB);
	cv::cvtColor(imageRGB, imageLab, COLOR_RGB2Lab);
	cv::split(imageLab, lab);
	cv::split(imageLab, lab_a);
	cv::split(imageLab, lab_b);

	cv::meanStdDev(lab[0],mean[0],standardDeviation[0]);
	cv::meanStdDev(lab[1],mean[1],standardDeviation[1]);
	cv::meanStdDev(lab[2],mean[2],standardDeviation[2]);
	// l-l
	// std::cout<<mean[2]<< " mean"<<std::endl;
	cv::subtract(lab[0],mean[0][0],lab[0]);
	cv::subtract(lab[1],mean[0][0],lab[1]);
	cv::subtract(lab[2],mean[0][0],lab[2]);
	// l-a
	cv::subtract(lab_a[0],mean[1][0],lab_a[0]);
	cv::subtract(lab_a[1],mean[1][0],lab_a[1]);
	cv::subtract(lab_a[2],mean[1][0],lab_a[2]);
	// //l-b
	cv::subtract(lab_b[0],mean[2][0],lab_b[0]);
	cv::subtract(lab_b[1],mean[2][0],lab_b[1]);
	cv::subtract(lab_b[2],mean[2][0],lab_b[2]);

	// l-l
	cv::pow(lab[0],2.0,lab[0]);
	cv::pow(lab[1],3.0,lab[1]);
	cv::pow(lab[2],3.0,lab[2]);
	
	// // l-a
	cv::pow(lab_a[0],2.0,lab_a[0]);
	cv::pow(lab_a[1],3.0,lab_a[1]);
	cv::pow(lab_a[2],3.0,lab_a[2]);
	// // l-b
	cv::pow(lab_b[0],2.0,lab_b[0]);
	cv::pow(lab_b[1],3.0,lab_b[1]);
	cv::pow(lab_b[2],3.0,lab_b[2]);

	// l-l
	cv::normalize(lab[0],lab[0],0.0,255.0,NORM_MINMAX,CV_8UC1);
	cv::normalize(lab[1],lab[1],0.0,255.0,NORM_MINMAX,CV_8UC1);
	cv::normalize(lab[2],lab[2],0.0,255.0,NORM_MINMAX,CV_8UC1);

	// // l-a
	cv::normalize(lab_a[0],lab_a[0],0.0,255.0,NORM_MINMAX,CV_8UC1);
	cv::normalize(lab_a[1],lab_a[1],0.0,255.0,NORM_MINMAX,CV_8UC1);
	cv::normalize(lab_a[2],lab_a[2],0.0,255.0,NORM_MINMAX,CV_8UC1);
	// // l-b
	cv::normalize(lab_b[0],lab_b[0],0.0,255.0,NORM_MINMAX,CV_8UC1);
	cv::normalize(lab_b[1],lab_b[1],0.0,255.0,NORM_MINMAX,CV_8UC1);
	cv::normalize(lab_b[2],lab_b[2],0.0,255.0,NORM_MINMAX,CV_8UC1);

	cv::merge(lab, imageLab);
	cv::merge(lab_a, imageLab_a);
	cv::merge(lab_b, imageLab_b);

	cv::addWeighted(imageLab_a, .5, imageLab, .5, 0.0, addWt1);
	cv::addWeighted(imageLab_b, .5, imageLab, .5, 0.0, addWt11);
	cv::addWeighted(addWt1, .5, addWt11, .5, 0.0, imageLab);

	cv::cvtColor(imageLab, imageFilter, COLOR_RGB2GRAY);
	cv::cvtColor(addWt1, imageFilter2, COLOR_RGB2GRAY);
	cv::cvtColor(addWt11, imageFilter3, COLOR_RGB2GRAY);
	// std::cout<<"HERE "<<imageFilter.at<cv::Vec3b>(4,5)<<std::endl;

	cv::vector<cv::vector<cv::Point> > contours;
	cv::vector<cv::Vec4i> hierarchy	;

	cv::bilateralFilter(imageFilter, imageFilterN, 2,100,100);
	cv::adaptiveThreshold(imageFilterN, imageFilterN,255,cv::ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY,5,-5);
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3), cv::Point(-1,-1));
	cv::dilate(imageFilterN, imageFilterN, element );
	cv::erode( imageFilterN, imageFilterN, element );
	cv::Mat imageCopy = imageFilterN.clone();

	cv::findContours(imageCopy,contours,hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE,cv::Point(0,0));
	cv::vector<cv::vector <cv::Point> > contours_poly(contours.size());
	cv::vector<cv::Rect> boundRect(contours.size());
	cv::Mat drawing = cv::Mat::zeros( inImage.size(), CV_8UC3 );
	cv::vector<int> width, height;
	cv::vector<cv::Point2f> points;	
	cv::vector<cv::RotatedRect> minRect( contours.size() );
	geometry_msgs::Point rwc;
	geometry_msgs::Point realWorldCoords;
	realWorldCoords.x = 0;
	realWorldCoords.y=0;
	std_msgs::Float64 orientation;
	rwc.x = 0;
	rwc.y = 0;
	int itr=0;
	for(unsigned int i = 0; i< contours.size(); i++){
		double area =cv::contourArea(contours[i], true);
		if (std::abs(area)>400 & std::abs(area)<800){
			cv::Moments moment = cv::moments((cv::Mat)contours[i]);
			std::cout<<moment.m00<<" are"<<std::endl;

			boundRect[i] = cv::boundingRect(contours[i]);
			float w = boundRect[i].width, h = boundRect[i].height;
			// std::cout<< " w "<< w<< " h "<<h<<std::endl;
			if ( w/h>.90 and h/w<1.09 and h>20.0 and h<40.0 and w>20.0 and w<40.0){
			points.push_back(cv::Point2f(moment.m10 / moment.m00, moment.m01 / moment.m00) );
			// cout<<points[i]<<endl;
			// cout<< float(points[i].x) <<float(points[i].y) <<endl;
			cv::circle(inImage, points.back(), 3, cv::Scalar(0,0,255), -1, 8, 0);
			// cout<<boundRect[i].width<< endl;
			cv::rectangle( inImage, boundRect[i].tl(), boundRect[i].br(), cv::Scalar(0,0,255), 1, 8, 0 );
			cv::drawContours( inImage,contours, i, cv::Scalar(255,0,0), 1, 8, hierarchy, 0, cv::Point() );
            // drawContours( inImage, hull, i, Scalar(0,0,0),1, 8, vector<Vec4i>(), 0, Point() );
			minRect[i] = cv::minAreaRect(contours[i]);
			// vector<float> angle;
			// angle.push_back(minRect[i].angle);
			// std::cout<<angle[0]<<" degrees"<<std::endl;
			// cv::Point2f rect_points[4]; minRect[i].points( rect_points );
			// for( int j = 0; j < 4; j++ )
				// cv::line( drawing, rect_points[j], rect_points[(j+1)%4], cv::Scalar(255,255,255), 4, 8 );

			// orientation.data = angle;
			rwc.x = ((points[itr].x-px*.5)/pixPerUnit);
			rwc.y = ((py*.5-points[itr].y)/pixPerUnit);
			rwc.z = height_camera;
			flag = flag + 1;

			// cout<<"Contour centre pts "<<points[itr]<<endl;
			// cout<<"Real world coord "<<rwc<<endl;
			// cout<<"pix per unit "<<pixPerUnit<<endl;
			// cout<<"height is "<<height_camera*pixPerUnit<<endl;
			// m_xyzPlacePub.publish(rwc);
			}
			cout<<"flag "<<flag<<endl;
			if(flag == 40)
			{
			   realWorldCoords.x = tf.x + (rwc.x/100);
			   realWorldCoords.y = tf.y + (rwc.y/100);
			   cout<<"Real world coords "<<realWorldCoords<<endl;
			   m_xyzPlacePub.publish(realWorldCoords);
			   break;
			   // m_orientationPub.publish(orientation);
			}
		}
	}
	cv::imshow("contours", inImage);
	cv::waitKey(1);
}
int main(int argc, char** argv) {
	ros::init(argc, argv, "chip_place_node");
	ros::NodeHandle nh;	
	BGAPlace place(nh);
	ros::spin();
}