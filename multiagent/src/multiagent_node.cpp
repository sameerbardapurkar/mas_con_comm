#include <multiagent/headers.h>
#include <multiagent/master.h>
#include <multiagent/robot.h>
using namespace std;

master::master()
{
  ros::NodeHandle nh("~");
  nh.param("total_robots",total_robots,0);
  nh.param("true_map_topic",true_map_topic,std::string("/truemap"));
  nh.param("common_map_topic",common_map_topic,std::string("/commonmap"));

  truemap = *(ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(true_map_topic));
  map = *(ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(common_map_topic));

  pubdata();
}
void master::pubdata()
{
  ROS_INFO("Initialized master with %d robots",total_robots);
}
int main(int argc, char **argv)
{
  ros::init(argc,argv,"master");
  master multiagent_controller;
  robot robot;
  return 0;
}
