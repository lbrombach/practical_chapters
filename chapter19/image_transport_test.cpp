#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

//using namespace std;

image_transport::Subscriber image_sub;
image_transport::Publisher image_pub;

void handle_image(const sensor_msgs::ImageConstPtr &img)
{
    //declare a new image
    sensor_msgs::Image output_image;

    //simply copy all the data fields from the recieved image
    output_image.header = img->header;
    output_image.width = img->width;
    output_image.height = img->height;
    output_image.encoding = img->encoding;
    output_image.is_bigendian = img->is_bigendian;
    output_image.step = img->step;
    output_image.data = img->data;

    //publish our ros image message
    image_pub.publish(output_image);
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
    while (ros::ok)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}