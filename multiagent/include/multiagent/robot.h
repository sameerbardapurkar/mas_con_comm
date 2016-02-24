class robot{
public:
  void setMap(nav_msgs::OccupancyGrid map);
  void makePlan();
protected:
  nav_msgs::OccupancyGrid map;
};
