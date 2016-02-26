#include <multiagent/multiagent.h>

using namespace std;

multiagent::multiagent()
{
  ros::NodeHandle nh("~");
  nh.param("total_robots",total_robots,0);
  nh.param("true_map_topic",true_map_topic,std::string("/truemap"));
  nh.param("common_map_topic",common_map_topic,std::string("/commonmap"));
  nh.param("config_file",config_file,std::string(""));
  nh.param("planner_epsilon",plannerEpsilon,1.0);
  nh.param("allocated_time",allocatedTime,5.0);
  nh.param("stop_after_first_solution",stopAfterFirstSolution,true);
  nh.param("backward_search",backwardSearch,false);

  const char* fname = config_file.c_str();

  truemap = *(ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(true_map_topic));
  map = *(ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(common_map_topic));

 multiagent::experiment_config config;
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
  robots_.resize(total_robots);
  startPosePublisher_.resize(total_robots);
  goalPosePublisher_.resize(total_robots);
  currentPosePublisher_.resize(total_robots);
  mapPublisher_.resize(total_robots);
  plannedPathPublisher_.resize(total_robots);
  traversedPathPublisher_.resize(total_robots);
  polygonPublisher_.resize(total_robots);
  populateRobots(config);
  simulate();

}
void multiagent::simulate()
{
  int count = 0;
  while(count<100)
  {
    ROS_INFO("Step: %d",count);
    for(int i=0; i<total_robots; i++)
    {
      robots_[i].updateEnv(robots_[i].getCurrent(),robots_[i].getGoal(), robots_[i].getMap());
      double plan_cost = robots_[i].makePlan();
      ROS_INFO("Plan for robot %d costs %f",i,plan_cost);
      currentPosePublisher_[i].publish(robots_[i].getCurrentPose());
      startPosePublisher_[i].publish(robots_[i].getStartPose());
      goalPosePublisher_[i].publish(robots_[i].getGoalPose());
      plannedPathPublisher_[i].publish(robots_[i].getPlannedPath());
      traversedPathPublisher_[i].publish(robots_[i].getTraversedPath());
      mapPublisher_[i].publish(robots_[i].getMap());
      robots_[i].advanceRobot();
      //char poseTopic[80];
      //sprintf(poseTopic,"robot_%d",i);
      //posePublisher = nh.advertise<geometry_msgs::PoseStamped>(poseTopic,1000);
      ros::Duration(0.25).sleep();
    }
    count++;
  }
}

void multiagent::populateRobots(multiagent::experiment_config config)
{
  for(int i=0; i<total_robots; i++)
  {
    std::vector<double> start;
    std::vector<double> goal;
    std::vector<sbpl_2Dpt_t> footprint;
    std::string mprimfile;
    start.push_back(config.start_x[i]);
    start.push_back(config.start_y[i]);
    start.push_back(config.start_theta[i]);
    goal.push_back(config.goal_x[i]);
    goal.push_back(config.goal_y[i]);
    goal.push_back(config.goal_theta[i]);
    mprimfile = config.mprim_filenames[i];
    footprint = config.footprints[i];

    robots_[i].Initialize(i,start,goal,map,truemap,mprimfile,footprint,plannerEpsilon,allocatedTime,stopAfterFirstSolution,backwardSearch);
    char currentPoseTopic[80];
    sprintf(currentPoseTopic,"robot_%d/current_pose",i);
    currentPosePublisher_[i] = nh.advertise<geometry_msgs::PoseStamped>(currentPoseTopic,1000);
    char startPoseTopic[80];
    sprintf(startPoseTopic,"robot_%d/start_pose",i);
    startPosePublisher_[i] = nh.advertise<geometry_msgs::PoseStamped>(startPoseTopic,1000);
    char goalPoseTopic[80];
    sprintf(goalPoseTopic,"robot_%d/goal_pose",i);
    goalPosePublisher_[i] = nh.advertise<geometry_msgs::PoseStamped>(goalPoseTopic,1000);
    char mapTopic[80];
    sprintf(mapTopic,"robot_%d/map",i);
    mapPublisher_[i] = nh.advertise<nav_msgs::OccupancyGrid>(mapTopic,1000);
    char plannedPathTopic[80];
    sprintf(plannedPathTopic,"robot_%d/planned_path",i);
    plannedPathPublisher_[i] = nh.advertise<nav_msgs::Path>(plannedPathTopic,1000);
    char traversedPathTopic[80];
    sprintf(traversedPathTopic,"robot_%d/traversed_path",i);
    traversedPathPublisher_[i] = nh.advertise<nav_msgs::Path>(traversedPathTopic,1000);
    char polygonTopic[80];
    sprintf(polygonTopic,"robot_%d/footprint",i);
    polygonPublisher_[i] = nh.advertise<geometry_msgs::Polygon>(polygonTopic,1000);
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
