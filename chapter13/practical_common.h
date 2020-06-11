#ifndef PRACTICAL_COMMON_H_
#define PRACTICAL_COMMON_H_
#include "nav_msgs/OccupancyGrid.h"
#include <math.h>
#include<iostream>



//cells from other sources set above this will be considered 100% occupied
const int OCCUPIED_THRESHOLD = 50;

// distance from center of robot to furthest exterior point, in meters
const double ROBOT_RADIUS = .2;


//3 helper functions to get x, y coordinates from index,
//and index from x, y coordinates, based on  index = ogm.info.width * y + x
int getX(int index, const nav_msgs::OccupancyGridPtr &map)
{
    return index % map->info.width;
}
int getY(int index, const nav_msgs::OccupancyGridPtr &map)
{
    return index / map->info.width;
}
int getIndex(int x, int y, const nav_msgs::OccupancyGridPtr &map)
{
    return map->info.width * y + x;
}


//helper function to stay in map boundaries and avoid segfaults
bool is_in_bounds(int x, int y, const nav_msgs::OccupancyGridPtr &map)
{
    return (x >= 0 && x < map->info.width && y >= 0 && y < map->info.height);
}

//helper to check if cell is marked unknown
bool is_unknown(int x, int y, const nav_msgs::OccupancyGridPtr &map)
{
    return ((int)map->data[getIndex(x, y, map)] == -1);
}


//helper to check if cell is to be considered an obstacle - includes cells marked unknown
bool is_obstacle(int x, int y, const nav_msgs::OccupancyGridPtr &map)
{
 //   std::cout<<x<<", "<<y<< " .... "<<(int)map->data[getIndex(x, y, map)]<<std::endl;
    return ((int)map->data[getIndex(x, y, map)] > OCCUPIED_THRESHOLD) || is_unknown(x, y, map);
}


//helper to return map resolution
double map_resolution(const nav_msgs::OccupancyGridPtr &map)
{
    return map->info.resolution;
}

//returns slope m from slope intercept formula y=m*x+b from two coordinate pairs
//don't forget all coordinates must be either pose in meters or grid cells numbers
// ( grid cell number = pose(in meters) / map_resolution  )
//DO NOT MIX POSE COORDINATES WITH GRID CELL COORDINATES - MAKE ALL THE SAME
double get_m(double x1, double y1, double x2, double y2)
{
    //****CAUTION< WILL THROW ERROR IF WE DIVIDE BY ZERO
    return (y1 - y2) / (x1 - x2);
}

// b as is the offset from slope intercept formula y=m*x+b
//for b = y-(m*x)
//DO NOT MIX POSE COORDINATES WITH GRID CELL COORDINATES - MAKE ALL THE SAME
double get_b(double x1, double y1, double x2, double y2)
{
  //  cannot divide by zer0
    if(x1 != x2)
    {
    return y1 - (get_m(x1, y1, x2, y2) * x1);
    }
    else return x1; //line is vertical, so b = x1
}

//returns where y falls on a line between two supplied points, for the given x
//returns Y from slope intercept forumula y=m*x+b, given x
//DO NOT MIX POSE COORDINATES WITH GRID CELL COORDINATES - MAKE ALL THE SAME
//****DOES NOT HANDLE VERTICAL LINES****
double get_y_intercept(double x1, double y1, double x2, double y2, double checkX)
{
    double m = get_m(x1, y1, x2, y2);
    double b = get_b(x1, y1, x2, y2);
    return m * checkX + b;
}

//returns where y falls on a line between two supplied points, for the given y
//returns x from slope intercept forumula y=m*x+b, given y. for x= (y-b)/m
//DO NOT MIX POSE COORDINATES WITH GRID CELL COORDINATES - MAKE ALL THE SAME
//****DOES NOT HANDLE VERTICAL LINES****
double get_x_intercept(double x1, double y1, double x2, double y2, double checkY)
{

    double m = get_m(x1, y1, x2, y2);
    double b = get_b(x1, y1, x2, y2);
    return (checkY - b) / m;
}


#endif /* PRACTICAL_COMMON_H_ */
