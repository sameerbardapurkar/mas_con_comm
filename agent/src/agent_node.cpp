#include<agent/agent_node.h>
#include<agent/robot.h>
using namespace std;

agent::agent()
{
  ros::NodeHandle nh("~");
  nh.param("agent_id",agent_id,0);
  nh.param("true_map_topic",true_map_topic,std::string("/truemap"));
  nh.param("common_map_topic",common_map_topic,std::string("/commonmap"));

  truemap = *(ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(true_map_topic));
  map = *(ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(common_map_topic));

  pubdata();
}
void agent::pubdata()
{
  ROS_INFO("Node with ID %d is active",agent_id);
}
int main(int argc, char **argv)
{
  ros::init(argc,argv,"agent");
  robot robot;
  return 0;
}
