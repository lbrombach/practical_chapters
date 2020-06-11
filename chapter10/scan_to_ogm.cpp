/*
*scan_to_ogm_beta.cpp
*
*This is a simple ROS node that accepts synchronized laser scan and pose messages to build an
*occupancy grid map and publish that map message. This is an example program to accompany chapter 10 
*of the book Practical Robotics in C++ by Lloyd Brombach and published by Packt Publishing.
*
*Author: Lloyd Brombach (lbrombach2@gmail.com)
*10/24/2019
*/

#include "ros/ros.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <cmath>

using namespace std;
using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace message_filters;

//manual offset for laser position. Better to handle with transforms
double transformX = .1;

//map metadata
const int width = 100;
const int height = 100;
const double resolution = .1;

const int MIN_INTENSITY = 45;

ros::Publisher pubMap;
nav_msgs::OccupancyGrid::Ptr mapPtr(new nav_msgs::OccupancyGrid());


//helper functions to make sure we iterate from low to high in marking
double getStart(double collision, double currentPose, double border = 0)
{
    double start = (currentPose < collision) ? currentPose : collision;
    start = (start < 0) ? 0 : start;
    return start;
}
double getEnd(double collision, double currentPose, double border)
{
    double end = (currentPose < collision) ? collision : currentPose;
    end = (end > border) ? border : end;
    return end;
}

//decreases occupancy value of cells that fall between laser and detected obstacle
void mark_free(double xCollision, double yCollision, double scanAngle, const PoseStampedConstPtr &pose)
{
    mapPtr->data[mapPtr->info.width * (int)pose->pose.position.y*10 + int(pose->pose.position.x)*10] = 0;
    double m = -1;
    double b = -1;
    int start = 0;
    int last = 0;
    int ogmIndex = 0;
    int y = 0;
    // cannot divide by zero, vertical line so mark every cell free until hit
    if (pose->pose.position.x == xCollision)
    {
        for (int y = (int)(getStart(yCollision, pose->pose.position.y, 0) * resolution); y < (int)(getEnd(yCollision, pose->pose.position.y, height - 1) * resolution); y++)
        {
            // decrement is by four. Make sure we don't go below zero
                mapPtr->data[mapPtr->info.width * y + (int)(pose->pose.position.x * resolution)] -= 15;
                mapPtr->data[mapPtr->info.width * y + (int)(pose->pose.position.x * resolution)] =
                       (mapPtr->data[mapPtr->info.width * y + (int)(pose->pose.position.x * resolution)] <0) ?
                       0 : mapPtr->data[mapPtr->info.width * y + (int)(pose->pose.position.x * resolution)];
        }
    }
    else
    {
        /*
        /checks y coordinate at every .5 resolution * x val. Decrements cells on that lines closer to "free" until
        /we hit border or obstacle cell. Here we start by calulating y=m*x+b slope and b values
        */
        m = (pose->pose.position.y - yCollision) / (pose->pose.position.x - xCollision);
        b = (pose->pose.position.y / resolution * 10) - (m * pose->pose.position.x / resolution * 10);

        //order for iterating. Everything multiplied by 10 so we can iterate at higher resolution than 1x per cell
        start = getStart(xCollision, pose->pose.position.x, 0) / resolution * 10;
        last = getEnd(xCollision, pose->pose.position.x, width / resolution) / resolution * 10;
        //iterate every .5 cell widths. Decrease occupancy value by 4 per scan
        for (int x = start; x < last; x += 5)
        {
            y = m * x + b;
            ogmIndex = (mapPtr->info.width * (y / 10)) + (x / 10);
            if (y > 0 && y / 10 < mapPtr->info.width)
            {
                mapPtr->data[ogmIndex] -= 15;
                mapPtr->data[ogmIndex] = (mapPtr->data[ogmIndex] < 0) ? 0 : mapPtr->data[ogmIndex];
            }
        }
    }
}


//callback function that happens whenwe recieve pose with close enough time stamp to scan
//ranges are in meters. cells are .1 meters so we mulitply hits by 10 to identify cell x and y
void scan_handler(const PoseStampedConstPtr &pose, const LaserScanConstPtr &scan)
{
    int xHit = -1;
    int yHit = -1;
    for (int i = 0; i < scan->ranges.size(); i++)
    {
        //look for objects detected over every scan index when intesity is high enough
        if (scan->intensities[i] > MIN_INTENSITY )
        {
            double scanAngle = pose->pose.orientation.z + scan->angle_min + (i * scan->angle_increment);
            double xDistance = cos(scanAngle) * scan->ranges[i];
            double yDistance = sin(scanAngle) * scan->ranges[i];
            double xCollision = xDistance + pose->pose.position.x;
            double yCollision = yDistance + pose->pose.position.y;
            //mark free whatever is between laser and object detected
            //x and y map coordinates of detection
            xHit = int(xCollision * 10);
            yHit = int(yCollision * 10);
            //increase occupancy value by 20 whatever cell the object is detected in to a max value of 100
            if (xHit >= 0 && xHit < width && yHit >= 0 && yHit < height && mapPtr->data[mapPtr->info.width * yHit + xHit] <= 100&& scan->ranges[i] > scan->range_min && scan->ranges[i] < scan->range_max )
            {
                mapPtr->data[mapPtr->info.width * yHit + xHit] += 25;
             if(mapPtr->data[mapPtr->info.width * yHit + (xHit+1)]<mapPtr->data.size())
                mapPtr->data[mapPtr->info.width * yHit + (xHit+1)] += 25;

                mapPtr->data[mapPtr->info.width * yHit + xHit] = (mapPtr->data[mapPtr->info.width * yHit + xHit] > 100) ? 100 : mapPtr->data[mapPtr->info.width * yHit + xHit];
            }
            if(scan->ranges[i] > scan->range_min && scan->ranges[i] < scan->range_max)
            {
            mark_free(xCollision, yCollision, scanAngle, pose);
            }

        }
    }
    mapPtr->header.stamp = ros::Time::now();
    pubMap.publish(mapPtr);
}

int main(int argc, char **argv)
{

    mapPtr->info.width = width;
    mapPtr->info.height = height;
    mapPtr->info.resolution = resolution;

    mapPtr->info.origin.position.x = 0;
    mapPtr->info.origin.position.y = 0;
    //init all cells to unknown
    mapPtr->data.resize(mapPtr->info.width * mapPtr->info.height);
    for (int i = 0; i < mapPtr->data.size(); i++)
    {
        mapPtr->data[i] = -1;
    }


    ros::init(argc, argv, "simple_OGM_Pub");
    ros::NodeHandle node;
    //use filters and approximate sync to run callack when timestamps are close enough
    message_filters::Subscriber<PoseStamped> pose_sub(node, "pose", 1);
    message_filters::Subscriber<LaserScan> scan_sub(node, "scan", 1);

    typedef sync_policies::ApproximateTime<PoseStamped, LaserScan> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pose_sub, scan_sub);
    sync.registerCallback(boost::bind(&scan_handler, _1, _2));

    //advertise publisher
    pubMap = node.advertise<nav_msgs::OccupancyGrid>("map", 0);

    while (ros::ok)
    {
        ros::spin();
    }

    return 0;
}
