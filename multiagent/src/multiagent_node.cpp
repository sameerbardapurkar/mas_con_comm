#include <multiagent/multiagent.h>

using namespace std;

multiagent::multiagent()
{
  ros::NodeHandle nh("~");
  nh.param("total_robots",total_robots,0);
  nh.param("true_map_topic",true_map_topic,std::string("/truemap"));
  nh.param("common_map_topic",common_map_topic,std::string("/commonmap"));

  truemap = *(ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(true_map_topic));
  map = *(ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(common_map_topic));

  robots.resize(total_robots);
  populateRobots();
}
void multiagent::populateRobots()
{
  robotClass robot;
  for(int i=0; i<total_robots; i++)
  {
    robotClass robot;
    robots[i].setMap(map);
  }
}
int main(int argc, char **argv)
{
  ros::init(argc,argv,"master");
  multiagent multiagent_controller;
  robotClass robot;
  return 0;
}
