#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;

int main(int argc, char** argv) {

    ros::init(argc, argv, "camera_capture_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);

    cv::VideoCapture cap;
    cap.open(1);
    // give webcam some time
    // std::this_thread::sleep_for(std::chrono::milliseconds(200));
    // Check if video device can be opened with the given index
    if (!cap.isOpened())
        return 1;
    sensor_msgs::ImagePtr msg;
    ROS_INFO("Webcam publisher running");

    ros::Rate loop_rate(10); 
    while (nh.ok()){
    // for(int iter = 0; iter<200; iter++){

        cv::Mat frame;
        cap >> frame;
        cout<<"frame type: "<<frame.type()<<"\n";
            cout<<"frame empty: "<<frame.empty()<<"\n";
        // Check if grabbed frame is actually full with some content
        if (!frame.empty()) {
        	cout<< " inner "<<endl;
            msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", frame).toImageMsg();
            pub.publish(msg);
            cv::imshow("Camera",frame);
            cv::waitKey(1);
        }
        // cout<<"counter "<<iter<<endl;
        ros::spinOnce();
        loop_rate.sleep();
    }
        cap.release();
}