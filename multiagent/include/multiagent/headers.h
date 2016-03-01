#ifndef _MASTER
#define _MASTER
#include<iostream>
#include<string>
#include<fstream>

#ifndef ROS
#define ROS
#endif
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PolygonStamped.h>



#include <sbpl/discrete_space_information/environment_navxythetalat.h>
#include <sbpl/headers.h>

#endif
