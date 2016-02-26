#include<multiagent/headers.h>

class robotClass{
public:
  void setMap(nav_msgs::OccupancyGrid OccupancyMap, nav_msgs::OccupancyGrid trueOccupancyMap);
  double makePlan();
  void setStart(std::vector<double> location);
  void setGoal(std::vector<double> location);
  void setCurrent(std::vector<double> location);
  void setPrims(std::string filename);
  void setRobotPerimeter(std::vector<sbpl_2Dpt_t> footprint);
  void Initialize(int i, std::vector<double> start, std::vector<double> goal, nav_msgs::OccupancyGrid map, nav_msgs::OccupancyGrid truemap, std::string mprimfile, std::vector<sbpl_2Dpt_t> perimeterptsV, double epsilon, double allocated_time, bool firstsolution, bool backwardsearch);
  void updateEnv(std::vector<double> start, std::vector<double> goal, nav_msgs::OccupancyGrid map);
  void initEnv();
  void setPlannerParams(double epsilon, double allocated_time, bool firstsolution, bool backwardsearch);
  void advanceRobot();
  void revealMap();
  void initializePlanner();
  std::vector<double> stateIDtoXYCoord(int state_id);
  std::vector<double> getStart();
  std::vector<double> getCurrent();
  std::vector<double> getGoal();
  nav_msgs::OccupancyGrid getMap();
  geometry_msgs::PoseStamped locationToPose(std::vector<double> location);
  geometry_msgs::PoseStamped getCurrentPose();
  geometry_msgs::PoseStamped getStartPose();
  geometry_msgs::PoseStamped getGoalPose();
  nav_msgs::Path getPlannedPath();
  nav_msgs::Path getTraversedPath();
protected:
  //Robot related stuff
  int id;
  //Map Related Variables
  nav_msgs::OccupancyGrid map;
  nav_msgs::OccupancyGrid truemap;
  double start_x;
  double start_y;
  double start_theta;
  std::vector<double> start;

  //Goal-Start-Position Related Variables
  double goal_x;
  double goal_y;
  double goal_theta;
  std::vector<double> goal;

  double current_x;
  double current_y;
  double current_theta;
  std::vector<double> current;

  //Motion Primitive Variables
  std::string mprimfile;
  double nominal_vel;
  double timetoturn45deg;

  //Costmap level stuff
  int obsthresh;
  int cost_inscribed;
  int cost_circumscribed;

  //SBPL related stuff
  double cellsize;
  int theta_disc;
  std::vector<sbpl_2Dpt_t> perimeterptsV;
  EnvironmentNAVXYTHETALAT env;
  double plannerEpsilon;
  double allocated_time_secs;
  int start_id;
  int goal_id;
  int current_id;
  std::vector<int> solution_state_IDs;
  int solcost;
  bool bSearchUntilFirstSolution;
  bool bBackwardSearch;
  SBPLPlanner* planner;
  std::vector< EnvNAVXYTHETALAT3Dpt_t > xythetaPath;
  std::vector<EnvNAVXYTHETALATAction_t> action_list;
  //Plan Specific Variables
  double cost_expended;

  //ROS_STUFF
  geometry_msgs::PoseStamped currentPose;
  nav_msgs::Path plannedPath;
  nav_msgs::Path traversedPath;
};
