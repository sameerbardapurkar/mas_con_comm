#include <multiagent/robotClass.h>

typedef std::vector<double> vec_d;

class multiagent{
protected:

  int total_robots;

  int total_cost;
  int communication_events;
  std::string outputFolder;
  std::string true_map_topic;
  std::string common_map_topic;
  std::string base_folder_;
  std::string testID;
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
      int num_tests;
      std::vector<std::string> mprim_filenames;
      std::vector< std::vector<sbpl_2Dpt_t> > footprints;
      std::string map_known;
      std::string map_unknown;
      int map_width;
      int map_height;
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
  ros::Publisher exptTrueMapPublisher_;
  double communicationEpsilon;
  ros::NodeHandle nh;
  bool communicationRequired;
  bool resetCostsRequired;

  bool populateRobots(const multiagent::experiment_config &config,
                      const std::vector<std::vector<double> > &starts,
                      const std::vector<std::vector<double> > &goals);
  void simulate();

  void set_up_experiment(const multiagent::experiment_config &config);
  std::vector<sbpl_2Dpt_t> get_footprint();
  bool is_exp_valid(int i, const multiagent::experiment_config &config, std::vector<double> start, std::vector<double> goal);
  void publish(int i);
  int takeStep();
  void mergeMaps();

public:
  int total_number_of_robots;
  multiagent();
  void start_experiments(const int num_starts_per_map,
                         const std::vector<int> map_ids,
                         const std::vector<int> all_num_robots, const std::vector<double> eps_array);
  void pubdata();
  bool get_exp_config_header(const char* filename,
                             multiagent::experiment_config& config) const;
  bool get_exp_config(const char* filename,
                      const int desired_test_number,
                      multiagent::experiment_config& config) const;
  void get_random_start_goal_pair(const int map_width,
                                   const int map_height,
                                   const int num_robots,
                                   std::vector<double> &start,
                                   std::vector<double> &goal) const;
};
