#ifndef _AGENT_NODE
#define _AGENT_NODE
#include<iostream>
#include<string>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/OccupancyGrid.h>

#include <sbpl/discrete_space_information/environment_navxythetalat.h>

#endif

class agent{
public:
  agent();
  void pubdata();

protected:
  int agent_id;
  int total_agents;
  std::string true_map_topic;
  std::string common_map_topic;

  nav_msgs::OccupancyGrid truemap;
  nav_msgs::OccupancyGrid map;

  ros::Publisher serial_no;
  ros::Publisher map_pub;

  ros::Subscriber map_sub;
};
