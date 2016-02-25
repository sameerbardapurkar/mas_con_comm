#include <multiagent/robotClass.h>
void robotClass::Initialize(int i, std::vector<double> start, std::vector<double> goal, nav_msgs::OccupancyGrid map, nav_msgs::OccupancyGrid truemap, std::string mprimfile, std::vector<sbpl_2Dpt_t> perimeterptsV, double epsilon, double allocated_time, bool firstsolution, bool backwardsearch)
{
  id = i;
  cost_expended = 0;
  setStart(start);
  setCurrent(start);
  setGoal(goal);
  setMap(map,truemap);
  setPrims(mprimfile);
  setRobotPerimeter(perimeterptsV);
  initEnv();
  setPlannerParams(epsilon,allocated_time,firstsolution,backwardsearch);
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

  double startx = start[0];
  double starty = start[1];
  double starttheta = start[2];

  double goalx = goal[0];
  double goaly = goal[1];
  double goaltheta = goal[2];

  const char* sMotPrimFile = mprimfile.c_str();
  const char* cost_possibly_circumscribed = std::string("cost_possibly_circumscribed_thresh").c_str();
  const char* cost_inscribed = std::string("cost_inscribed_thresh").c_str();

  env.SetEnvParameter(cost_possibly_circumscribed,100);
  env.SetEnvParameter(cost_inscribed,100);
  env.InitializeEnv (width, height, NULL/*mapdata, initializing to totally free map*/, startx, starty, starttheta, goalx, goaly, goaltheta, 0.1, 0.1, 0.1, perimeterptsV, cellsize_m, /*nominal_vel*/ 0.1, /*timetoturn45degsinplace_secs*/ 2, /*obsthresh*/ 100, sMotPrimFile);

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
void robotClass::setPlannerParams(double epsilon, double allocated_time, bool firstsolution, bool backwardsearch)
{
  plannerEpsilon = epsilon;
  allocated_time_secs = allocated_time;
  bSearchUntilFirstSolution = firstsolution;
  bBackwardSearch = backwardsearch;
}
double robotClass::makePlan()
{
  SBPLPlanner* planner = new ARAPlanner(&env, bBackwardSearch);
  planner->set_initialsolution_eps(plannerEpsilon);
  planner->set_search_mode(bSearchUntilFirstSolution);
  planner->set_start(start_id);
  planner->set_goal(goal_id);
  planner->replan(allocated_time_secs,&solution_state_IDs,&solcost);
  return(solcost);
}
void robotClass::advanceRobot()
{
  int travel_cost;
  std::vector<int> travel_state_IDs;
  env.ConvertStateIDPathintoXYThetaPath(&solution_state_IDs,&xythetaPath);
  std::vector<double> current_loc;
  current_loc[0] = xythetaPath[1].x;
  current_loc[1] = xythetaPath[1].y;
  current_loc[2] = xythetaPath[1].theta;
  setCurrent(current_loc);
  SBPLPlanner* cost_calc_planner = new ARAPlanner(&env, bBackwardSearch);
  cost_calc_planner->set_initialsolution_eps(plannerEpsilon);
  cost_calc_planner->set_search_mode(bSearchUntilFirstSolution);
  cost_calc_planner->set_start(solution_state_IDs[0]);
  cost_calc_planner->set_goal(solution_state_IDs[1]);
  cost_calc_planner->replan(allocated_time_secs,&travel_state_IDs,&travel_cost);
  cost_expended += travel_cost;
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
