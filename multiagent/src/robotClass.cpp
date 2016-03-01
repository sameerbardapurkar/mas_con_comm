#include <multiagent/robotClass.h>

void robotClass::Initialize(int i, std::vector<double> start, std::vector<double> goal, nav_msgs::OccupancyGrid map, nav_msgs::OccupancyGrid truemap, std::string mprimfile, std::vector<sbpl_2Dpt_t> perimeterptsV, double epsilon, double allocated_time, bool firstsolution, bool backwardsearch)
{
  id = i;
  cost_expended = 0;
  plannedPath.header.frame_id = std::string("map");
  traversedPath.header.frame_id = std::string("map");
  traversedPath.poses.clear();
  plannedPath.poses.clear();
  solution_state_IDs.clear();
  xythetaPath.clear();
  action_list.clear();
  setStart(start);
  //ROS_INFO("Initialized start state for robot with ID %d",i);
  setCurrent(start);
  //ROS_INFO("Initialized current state for robot with ID %d",i);
  //ROS_INFO("Setting goal for robot with ID %d",i);
  setGoal(goal);
  //ROS_INFO("Initialized goal state for robot with ID %d",i);
  setMap(map,truemap);
  map.data.assign(map.data.size(),0);
  //ROS_INFO("Initialized maps for robot with ID %d",i);
  setPrims(mprimfile);
  //ROS_INFO("Loaded motion primitives file for robot with ID %d, file is %s",i,(mprimfile.c_str()));
  setRobotPerimeter(perimeterptsV);
  //ROS_INFO("Initialized footprint for robot with ID %d",i);
  //ROS_INFO("Now initializing environment for robot with ID %d",i);
  initEnv();
  //ROS_INFO("..done");
  setPlannerParams(epsilon,allocated_time,firstsolution,backwardsearch);
  //ROS_INFO("calling initializePlanner");

  initializePlanner();
  //ROS_INFO("Initialized planner parameters for robot with ID %d",i);
  //ROS_INFO("Initialized robot with ID %d",i);
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
bool robotClass::updateEnv(std::vector<double> start, std::vector<double> goal, nav_msgs::OccupancyGrid map)
{
  //Now updating map
  int width = map.info.width;
  int height = map.info.height;
  for (int i=0;i<width;i++)
  {
    for (int j=0;j<height;j++)
    {
      env->UpdateCost(i,j,map.data[j*width + i]);
      if (map.data[j*width + i] == 100)
      {
        for(int k = i-5; k<i+5; k++)
        {
          for(int l = j-5; l<j+5; l++)
          {
            if(k>0&&k<width&&l>0&&l<height)
            {
              //ROS_INFO("hi");
              env->UpdateCost(k,l,std::max(int(100 - 10*abs(sqrt((k-i)*(k-i)+(l-j)*(l-j)))),int(map.data[l*width + k])));
            }

          }
        }
      }
    }
  }
  //updating start and goal
  ////ROS_INFO("Updating environment for robot %d",id);
  int sx,sy,st;
  int gx,gy,gt;
  env->PoseContToDisc(current[0],current[1],current[2],sx,sy,st);
  env->PoseContToDisc(goal[0],goal[1],goal[2],gx,gy,gt);
  if (env->IsValidConfiguration(sx,sy,st) && int(env->GetMapCost(sx,sy)) == 0)
  {

    start_id = env->SetStart(current[0],current[1],current[2]);
  }
  else
  {
    return false;
  }

  ////ROS_INFO("Setting start for robot %d",id);
  if (env->IsValidConfiguration(gx,gy,gt)&& int(env->GetMapCost(gx,gy)) == 0)
  {
    goal_id = env->SetGoal(goal[0],goal[1],goal[2]);
  }
  else
  {
    return false;
  }
  ////ROS_INFO("Setting goal for robot %d",id);
  if(start_id == -1 || goal_id == -1)
      return false;
  else
      return true;
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
  env.reset(new EnvironmentNAVXYTHETALAT);
  env->InitializeEnv (width, height, NULL/*mapdata, initializing to totally free map*/,
                                       /*startx*/0,/*starty*/ 0, /*starttheta*/0,
                                       /*goalx*/ 0,/*goaly*/ 0,/*goaltheta*/ 0,
                                       0.0, 0.0, 0.0, perimeterptsV, cellsize_m,
                                       /*nominal_vel*/ 0.1, /*timetoturn45degsinplace_secs*/ 2,
                                       /*obsthresh*/ 100, sMotPrimFile);
  env->SetEnvParameter(cost_possibly_circumscribed,100);
  env->SetEnvParameter(cost_inscribed,100);
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
  planner.reset(new ARAPlanner(env.get(), bBackwardSearch));
  planner->set_initialsolution_eps(plannerEpsilon);
  planner->set_search_mode(bSearchUntilFirstSolution);
}

bool robotClass::makePlan(int &solcost)
{
  //initializePlanner();
  planner->set_initialsolution_eps(plannerEpsilon);
  planner->set_search_mode(bSearchUntilFirstSolution);
  planner->force_planning_from_scratch();
  bool plan_success;
  std::vector<double> start_state_planner = stateIDtoXYCoord(start_id);
  std::vector<double> goal_state_planner = stateIDtoXYCoord(goal_id);
  //ROS_INFO("[planner] Setting start to start_id %d, with location of x:%f y:%f theta:%f"
      //  ,start_id,start_state_planner[0],start_state_planner[1],start_state_planner[2]);
  planner->set_start(start_id);
  //ROS_INFO("[planner] Setting goal to goal_id %d, with location of x:%f y:%f theta:%f"
      //  ,goal_id,goal_state_planner[0],goal_state_planner[1],goal_state_planner[2]);
  planner->set_goal(goal_id);
  //ROS_INFO("set_goal,now planning");
  solution_state_IDs.clear();
  plan_success = planner->replan(allocated_time_secs,&solution_state_IDs,&solcost);
  //ROS_INFO("Planning done");

  if (plan_success == true)
  {
    //ROS_INFO("Plan found for robot %d",id);
    if(solution_state_IDs.size() == 0)
    {
      //ROS_INFO("Invalid Path!");
    }
    for(int i = 0; i<solution_state_IDs.size(); i++)
    {
      //ROS_INFO("Solution state ID %d",solution_state_IDs[i]);
    }
    xythetaPath.clear();
    env->ConvertStateIDPathintoXYThetaPath(&solution_state_IDs,&xythetaPath);
    //ROS_INFO("Converted to xypath");
    for(int i = 0; i<xythetaPath.size(); i++)
    {
    //ROS_INFO("For robot %d, x:%f  y:%f  theta:%f",id, xythetaPath[i].x,xythetaPath[i].y,xythetaPath[i].theta);
    }
    env->GetActionsFromStateIDPath(&solution_state_IDs, &action_list);
    plannedPath.poses.clear();
    for(int i = 0; i<solution_state_IDs.size(); i++)
    {
      plannedPath.poses.push_back(locationToPose(stateIDtoXYCoord(solution_state_IDs[i])));
    }

  }
  else
  {
    //ROS_INFO("Failed to find plan");
  }
  //ROS_INFO("Sol cost %d",solcost);
  return plan_success;
}
int robotClass::advanceRobot()
{
  std::vector<double> current_loc = stateIDtoXYCoord(solution_state_IDs[1]);
  //ROS_INFO("Solution state ID to go to is %d",solution_state_IDs[1]);
  currentPose = locationToPose(current);
  traversedPath.poses.push_back(currentPose);
  //ROS_INFO("Advancing robot %d to x:%f y:%f theta:%f",id,current_loc[0],current_loc[1],current_loc[2]);
  setCurrent(current_loc);
  //ROS_INFO("Current pose of robot %d is x:%f y:%f theta:%f",id,current[0],current[1],current[2]);
  perimeterToFootprint();

  //ROS_INFO("Pose orientation is w:%f x:%f y:%f z:%f",currentPose.pose.orientation.w,currentPose.pose.orientation.x,currentPose.pose.orientation.y,currentPose.pose.orientation.z);
  revealMap();
  if(action_list.size()>0)
  {
    cost_expended += action_list[0].cost;
    return action_list[0].cost;
  }
  return 0;

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
        if(i>0&&i<map.info.width&&j>0&&j<map.info.height)
        {
          if((map.data[j*map.info.width + i] < truemap.data[j*map.info.width + i]))
          {
            map.data[j*map.info.width + i] = truemap.data[j*map.info.width + i];
          }
        }
      }
    }
  //ROS_INFO("Map Revealed for robot %d",id);
}
std::vector<double> robotClass::stateIDtoXYCoord(int state_id)
{
  std::vector<double> xypoint;
  int x_state, y_state, theta_state;
  double x_cont,y_cont,theta_cont;
  env->GetCoordFromState(state_id,x_state,y_state,theta_state);
  env->PoseDiscToCont(x_state,y_state,theta_state,x_cont,y_cont,theta_cont);
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
double robotClass::getExpendedCost()
{
  return cost_expended;
}
void robotClass::resetExpendedCost()
{
  cost_expended = 0;
}
double robotClass::getOriginalCost()
{
  return(original_cost);
}
void robotClass::setOriginalCost(int value)
{
  original_cost = value;
}
void robotClass::perimeterToFootprint()
{
  footprint.header.frame_id = std::string("map");
  footprint.polygon.points.clear();
  geometry_msgs::Point32 point;
  for(int i=0;i<perimeterptsV.size();i++)
  {
      point.x = std::cos(current[2])*perimeterptsV[i].x - std::sin(current[2])*perimeterptsV[i].y;
      point.y = std::sin(current[2])*perimeterptsV[i].x + std::cos(current[2])*perimeterptsV[i].y;
      point.z = 0;
      point.x = point.x + current[0];
      point.y = point.y + current[1];
      footprint.polygon.points.push_back(point);
  }
}
geometry_msgs::PolygonStamped robotClass::getFootprint()
{
  return(footprint);
}
