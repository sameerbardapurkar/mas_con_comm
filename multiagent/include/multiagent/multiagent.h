#include <multiagent/robotClass.h>
class multiagent{
protected:

  int total_robots;

  std::string true_map_topic;
  std::string common_map_topic;
  std::string config_file;

  nav_msgs::OccupancyGrid truemap;
  nav_msgs::OccupancyGrid commonmap;

  std::vector<robotClass> robots_;

  std::vector<nav_msgs::OccupancyGrid> robot_maps;
  ros::Publisher serial_no;
  ros::Publisher map_pub;

  ros::Subscriber map_sub;

  struct experiment_config
  {
      int num_robots;
      std::vector<std::string> mprim_filenames;
      std::vector< std::vector<sbpl_2Dpt_t> > footprints;
      std::vector<double> start_x;
      std::vector<double> start_y;
      std::vector<double> start_z;
      std::vector<double> start_theta;

      std::vector<double> goal_x;
      std::vector<double> goal_y;
      std::vector<double> goal_z;
      std::vector<double> goal_theta;
  };
  double plannerEpsilon;
  double allocatedTime;
  bool stopAfterFirstSolution;
  bool backwardSearch;
  std::vector<ros::Publisher> startPosePublisher_;
  std::vector<ros::Publisher> goalPosePublisher_;
  std::vector<ros::Publisher> currentPosePublisher_;
  std::vector<ros::Publisher> mapPublisher_;
  std::vector<ros::Publisher> plannedPathPublisher_;
  std::vector<ros::Publisher> traversedPathPublisher_;
  std::vector<ros::Publisher> polygonPublisher_;
  ros::Publisher commonMapPublisher_;
  double communicationEpsilon;
  ros::NodeHandle nh;
  bool communicationRequired;
  bool resetCostsRequired;


public:
  multiagent();
  void pubdata();
  void populateRobots(multiagent::experiment_config config);
  void simulate();
  bool get_exp_config(const char* filename,multiagent::experiment_config& config);
  std::vector<sbpl_2Dpt_t> get_footprint();
  void print_exp_config(const multiagent::experiment_config& config);
  void publish(int i);
  void takeStep();
  void mergeMaps();

};
