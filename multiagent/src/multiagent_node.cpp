#include <multiagent/multiagent.h>

using namespace std;

multiagent::multiagent()
{
  ros::NodeHandle nh("~");
  nh.param("total_robots",total_robots,0);
  nh.param("true_map_topic",true_map_topic,std::string("/truemap"));
  nh.param("common_map_topic",common_map_topic,std::string("/commonmap"));
  nh.param("config_file",config_file,std::string(""));
  const char* fname = config_file.c_str();

  truemap = *(ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(true_map_topic));
  map = *(ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(common_map_topic));

 experiment_config config;
 const bool succ = get_exp_config(fname, config);
 if (!succ)
 {
   ROS_ERROR("Error reading config.");
 }
 else
 {
   ROS_INFO("Loaded configuration successfully.");
   print_exp_config(config);
 }
  robots.resize(total_robots);
  populateRobots();
}
void multiagent::populateRobots()
{
  for(int i=0; i<total_robots; i++)
  {

    //robots[i].Initialize(i);
    //robots[i].setMap(map);

  }
}
int main(int argc, char **argv)
{
  ros::init(argc,argv,"master");
  multiagent multiagent_controller;
  robotClass robot;
  ros::spin();
  return 0;
}
