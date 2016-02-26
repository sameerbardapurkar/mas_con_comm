#include <multiagent/robotClass.h>
void robotClass::Initialize(int i, std::vector<double> start, std::vector<double> goal, nav_msgs::OccupancyGrid map, nav_msgs::OccupancyGrid truemap, std::string mprimfile, std::vector<sbpl_2Dpt_t> perimeterptsV, double epsilon, double allocated_time, bool firstsolution, bool backwardsearch)
{
  id = i;
  cost_expended = 0;
  plannedPath.header.frame_id = std::string("map");
  traversedPath.header.frame_id = std::string("map");
  setStart(start);
  //ROS_INFO("Initialized start state for robot with ID %d",i);
  setCurrent(start);
  //ROS_INFO("Initialized current state for robot with ID %d",i);
  setGoal(goal);
  //ROS_INFO("Initialized goal state for robot with ID %d",i);
  setMap(map,truemap);
  //ROS_INFO("Initialized maps for robot with ID %d",i);
  setPrims(mprimfile);
  //ROS_INFO("Loaded motion primitives file for robot with ID %d, file is %s",i,(mprimfile.c_str()));
  setRobotPerimeter(perimeterptsV);
  //ROS_INFO("Initialized footprint for robot with ID %d",i);
  ///ROS_INFO("Now initializing environment for robot with ID %d",i);
  initEnv();
  //ROS_INFO("..done");
  setPlannerParams(epsilon,allocated_time,firstsolution,backwardsearch);
  initializePlanner();
  //ROS_INFO("Initialized planner parameters for robot with ID %d",i);
  ROS_INFO("Initialized robot with ID %d",i);
}
void robotClass::setPrims(std::string filename)
{
  mprimfile = filename;
}
void robotClass::setRobotPerimeter(std::vector<sbpl_2Dpt_t> footprint)
{
  perimeterptsV = footprint;
}
void robotClass::setStart(std::vector<double> location)
{
  start = location;
  start_x = start[0];
  start_y = start[1];
  start_theta = start[2];
}
void robotClass::setGoal(std::vector<double> location)
{
  goal = location;
  goal_x = goal[0];
  goal_y = goal[1];
  goal_theta = goal[2];
}
void robotClass::setCurrent(std::vector<double> location)
{
  current = location;
  current_x = current[0];
  current_y = current[1];
  current_theta = current[2];
}
void robotClass::setMap(nav_msgs::OccupancyGrid OccupancyMap, nav_msgs::OccupancyGrid trueOccupancyMap)
{
  map = OccupancyMap;
  truemap = trueOccupancyMap;
}
void robotClass::updateEnv(std::vector<double> start, std::vector<double> goal, nav_msgs::OccupancyGrid map)
{
  int width = map.info.width;
  int height = map.info.height;
  //updating start and goal
  start_id = env.SetStart(current[0],current[1],current[2]);
  goal_id = env.SetGoal(goal[0],goal[1],goal[2]);
  //Now updating map
  for (int i=0;i<width;i++)
  {
    for (int j=0;j<height;j++)
    {
      if (map.data[j*width + i] == 100)
      {
        env.UpdateCost(i,j,100);
      }
    }
  }
}
void robotClass::initEnv()
{
  int width = map.info.width;
  int height = map.info.height;
  double cellsize_m = map.info.resolution;
  cellsize = cellsize_m;
  theta_disc = 8;
  const char* sMotPrimFile = mprimfile.c_str();
  const char* cost_possibly_circumscribed = std::string("cost_possibly_circumscribed_thresh").c_str();
  const char* cost_inscribed = std::string("cost_inscribed_thresh").c_str();
  env.SetEnvParameter(cost_possibly_circumscribed,100);
  env.SetEnvParameter(cost_inscribed,100);
  env.InitializeEnv (width, height, NULL/*mapdata, initializing to totally free map*/, /*startx*/0,/*starty*/ 0, /*starttheta*/0,/*goalx*/ 0,/*goaly*/ 0,/*goaltheta*/ 0, 0.0, 0.0, 0.0, perimeterptsV, cellsize_m, /*nominal_vel*/ 0.1, /*timetoturn45degsinplace_secs*/ 2, /*obsthresh*/ 100, sMotPrimFile);
}
void robotClass::setPlannerParams(double epsilon, double allocated_time, bool firstsolution, bool backwardsearch)
{
  plannerEpsilon = epsilon;
  allocated_time_secs = allocated_time;
  bSearchUntilFirstSolution = firstsolution;
  bBackwardSearch = backwardsearch;
}
void robotClass::initializePlanner()
{
  planner = new ARAPlanner(&env, bBackwardSearch);
  planner->set_initialsolution_eps(plannerEpsilon);
  planner->set_search_mode(bSearchUntilFirstSolution);
}
double robotClass::makePlan()
{
  bool plan_success;
  planner->set_start(start_id);
  planner->set_goal(goal_id);
  plan_success = planner->replan(allocated_time_secs,&solution_state_IDs,&solcost);
  env.ConvertStateIDPathintoXYThetaPath(&solution_state_IDs,&xythetaPath);
  env.GetActionsFromStateIDPath(&solution_state_IDs, &action_list);
  if (plan_success == true)
  {
    ROS_INFO("Plan found for robot %d",id);
    plannedPath.poses.clear();
    for(int i = 0; i<solution_state_IDs.size(); i++)
    {
      plannedPath.poses.push_back(locationToPose(stateIDtoXYCoord(solution_state_IDs[i])));
      //ROS_INFO("x:%f  y:%f  theta:%f",xythetaPath[i].x,xythetaPath[i].y,xythetaPath[i].theta);
    }
  }
  else
  {
    ROS_INFO("Failed to find plan");
  }
  return(solcost);
}
void robotClass::advanceRobot()
{
  std::vector<double> current_loc = stateIDtoXYCoord(solution_state_IDs[1]);
  currentPose = locationToPose(current);
  traversedPath.poses.push_back(currentPose);
  setCurrent(current_loc);
  ROS_INFO("Advancing robot %d to x:%f y:%f theta:%f",id,current_loc[0],current_loc[1],current_loc[2]);
  ROS_INFO("Pose orientation is w:%f x:%f y:%f z:%f",currentPose.pose.orientation.w,currentPose.pose.orientation.x,currentPose.pose.orientation.y,currentPose.pose.orientation.z);
  cost_expended += action_list[0].cost;
}
void robotClass::revealMap()
{
  //expose a 20by20 cell around the robot
  double cellsize = map.info.resolution;
  int row = int(current_x/cellsize);
  int col = int(current_y/cellsize);
  for (int i = row - 20; i<row + 20; i++)
    {
      for (int j = col - 20; j<col + 20; j++)
      {
        if((map.data[j*map.info.width + i] < truemap.data[j*map.info.width + i])&&(i>0&&i<map.info.width&&j>0&&j<map.info.height))
        {
          map.data[j*map.info.width + i] = truemap.data[j*map.info.width + i];
        }
      }
    }
}
std::vector<double> robotClass::stateIDtoXYCoord(int state_id)
{
  std::vector<double> xypoint;
  int x_state, y_state, theta_state;
  double x_cont,y_cont,theta_cont;
  env.GetCoordFromState(state_id,x_state,y_state,theta_state);
  x_cont = x_state*cellsize + 0.5*cellsize;
  y_cont = y_state*cellsize + 0.5*cellsize;
  theta_cont = theta_state*(M_PI/theta_disc);
  xypoint.push_back(x_cont);
  xypoint.push_back(y_cont);
  xypoint.push_back(theta_cont);
  return(xypoint);
}
std::vector<double> robotClass::getStart()
{
  return start;
}
std::vector<double> robotClass::getCurrent()
{
  return current;
}
std::vector<double> robotClass::getGoal()
{
  return goal;
}
nav_msgs::OccupancyGrid robotClass::getMap()
{
  return map;
}
geometry_msgs::PoseStamped robotClass::locationToPose(std::vector<double> location)
{
  geometry_msgs::PoseStamped robot_pose;
  robot_pose.header.frame_id = std::string("map");
  robot_pose.pose.position.x = location[0];
  robot_pose.pose.position.y = location[1];
  robot_pose.pose.position.z = 0;

  robot_pose.pose.orientation.w = std::cos(0.5*location[2]);
  robot_pose.pose.orientation.x = 0;
  robot_pose.pose.orientation.y = 0;
  robot_pose.pose.orientation.z = std::sin(0.5*location[2]);

  return(robot_pose);
}
geometry_msgs::PoseStamped robotClass::getCurrentPose()
{
  return locationToPose(current);
}
geometry_msgs::PoseStamped robotClass::getStartPose()
{
  return locationToPose(start);
}
geometry_msgs::PoseStamped robotClass::getGoalPose()
{
  return locationToPose(goal);
}
nav_msgs::Path robotClass::getPlannedPath()
{
  return plannedPath;
}
nav_msgs::Path robotClass::getTraversedPath()
{
  return traversedPath;
}
