class master{
public:
  master();
  void pubdata();

protected:
  
  int total_robots;
  std::string true_map_topic;
  std::string common_map_topic;

  nav_msgs::OccupancyGrid truemap;
  nav_msgs::OccupancyGrid map;

  std::vector<nav_msgs::OccupancyGrid> robot_maps;
  ros::Publisher serial_no;
  ros::Publisher map_pub;

  ros::Subscriber map_sub;
};
