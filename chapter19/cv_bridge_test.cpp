#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

using namespace std;
using namespace cv;

image_transport::Subscriber image_sub;
image_transport::Publisher image_pub;

void handle_image(const sensor_msgs::ImageConstPtr &img)
{
    //convert ros image message to open-cv compatible BGR image
    Mat image = cv_bridge::toCvShare(img, sensor_msgs::image_encodings::BGR8)->image;

    //draw a rectangle on image, originating at (150,150) or size 100x100,
    //set color to (255,0,0) (blue), and line width to 3.
    //  cv::rectangle(image, cv::Rect(150, 150, 100, 100), cv::Scalar(255, 0, 0), -1);
    cv::Rect rectangle(150, 150, 100, 100);
    cv::Scalar color(255, 0, 255);
    cv::rectangle(image, rectangle, color, -1);

    //use cv_bridge to convert opencv image to ros image message
    //create image message
    sensor_msgs::Image::Ptr output_img;
    //convert opencv image we drew on to ROS image message
    output_img = cv_bridge::CvImage(img->header, "bgr8", image).toImageMsg();
    //publish our ros image message
    image_pub.publish(output_img);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cv_bridge_test");
    ros::NodeHandle nh;
    image_transport::ImageTransport it_(nh);

    // Subscribe to input video feed and publish output video feed
    image_sub = it_.subscribe("/usb_cam/image_raw", 1, handle_image);
    image_pub = it_.advertise("image_output", 1);

    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}