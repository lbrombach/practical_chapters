#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "open_cv_test");
    ros::NodeHandle nh;

    //display Open CV version
    cout << "OpenCV version : " << CV_VERSION << endl;

    //create blank 3-color image of size 400x400pixels.
    cv::Mat testImage = cv::Mat::zeros(cv::Size(400, 400), CV_8UC3);

    //draw a rectangle on testImage, originating at (150,150) or size 100x100,
    //set color to (255,0,0) (blue), and line width to 3.
    cv::rectangle(testImage, cv::Rect(150,150,100,100), cv::Scalar(255, 0, 0), 3);

    //display testImage in a new window called test_window
    cv::imshow("test_window", testImage);

    //wait for key press tocontinue
    cv::waitKey(0);
    
    return 0;
}