#include<multiagent/headers.h>

class robotClass{
public:
  void setMap(nav_msgs::OccupancyGrid mapSet);
  void setStartGoal();
  void makePlan();
  void setEnv();
protected:
  nav_msgs::OccupancyGrid map;
  double start_x;
  double start_y;
  double start_theta;

  double goal_x;
  double goal_y;
  double goal_theta;

  double current_x;
  double current_y;
  double current_theta;

  EnvironmentNAVXYTHETALAT env;
};
