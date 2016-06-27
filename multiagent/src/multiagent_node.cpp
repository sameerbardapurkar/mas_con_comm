#include <multiagent/multiagent.h>
#include <multiagent/image_loader.h>
#include <ros/console.h>

using namespace std;

multiagent::multiagent()
{
  ros::NodeHandle nh("~");
  ros::Duration(3).sleep();
  nh.param("total_robots",total_number_of_robots,0);
  nh.param("base_folder", base_folder_, std::string(""));
  nh.param("true_map_topic",true_map_topic,std::string("/truemap"));
  nh.param("common_map_topic",common_map_topic,std::string("/commonmap"));
  nh.param("planner_epsilon",plannerEpsilon,1.0);
  nh.param("allocated_time",allocatedTime,5.0);
  nh.param("stop_after_first_solution",stopAfterFirstSolution,true);
  nh.param("backward_search",backwardSearch,false);
  nh.param("communication_epsilon",communicationEpsilon,100.0);
  nh.param("output_folder",outputFolder,std::string("/home/"));
  nh.param("test_id",testID,std::string("test"));
  robots_.clear();
  robots_.resize(total_number_of_robots);
  startPosePublisher_.resize(total_number_of_robots);
  goalPosePublisher_.resize(total_number_of_robots);
  currentPosePublisher_.resize(total_number_of_robots);
  mapPublisher_.resize(total_number_of_robots);
  plannedPathPublisher_.resize(total_number_of_robots);
  traversedPathPublisher_.resize(total_number_of_robots);
  polygonPublisher_.resize(total_number_of_robots);

  for(int i=0;i<total_number_of_robots;i++)
  {
    char currentPoseTopic[80];
    sprintf(currentPoseTopic,"eps%s/robot_%d/current_pose",testID.c_str(),i);
    currentPosePublisher_[i] = nh.advertise<geometry_msgs::PoseStamped>(currentPoseTopic,1000);
    char startPoseTopic[80];
    sprintf(startPoseTopic,"eps%s/robot_%d/start_pose",testID.c_str(),i);
    startPosePublisher_[i] = nh.advertise<geometry_msgs::PoseStamped>(startPoseTopic,1000);
    char goalPoseTopic[80];
    sprintf(goalPoseTopic,"eps%s/robot_%d/goal_pose",testID.c_str(),i);
    goalPosePublisher_[i] = nh.advertise<geometry_msgs::PoseStamped>(goalPoseTopic,1000);
    char mapTopic[80];
    sprintf(mapTopic,"eps%s/robot_%d/map",testID.c_str(),i);
    mapPublisher_[i] = nh.advertise<nav_msgs::OccupancyGrid>(mapTopic,1000);
    char plannedPathTopic[80];
    sprintf(plannedPathTopic,"eps%s/robot_%d/planned_path",testID.c_str(),i);
    plannedPathPublisher_[i] = nh.advertise<nav_msgs::Path>(plannedPathTopic,1000);
    char traversedPathTopic[80];
    sprintf(traversedPathTopic,"eps%s/robot_%d/traversed_path",testID.c_str(),i);
    traversedPathPublisher_[i] = nh.advertise<nav_msgs::Path>(traversedPathTopic,1000);
    char polygonTopic[80];
    sprintf(polygonTopic,"eps%s/robot_%d/footprint",testID.c_str(),i);
    polygonPublisher_[i] = nh.advertise<geometry_msgs::PolygonStamped>(polygonTopic,1000);
  }
  //truemap = *(ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(true_map_topic));
  //commonmap = *(ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(common_map_topic));
  //commonmap.data.assign(commonmap.data.size(),0);
  //ROS_INFO("SIZE OF MAP: %d", truemap.data.size());
}

void multiagent::start_experiments(const int num_starts_per_map,
                                   const std::vector<int> map_ids,
                                   const std::vector<int> num_robots, const std::vector<double> eps_array)
{
    // Each expt file is based on 1 map id and a fixed team of robots.
    std::ofstream dataDumper;
    std::string outputFilename;
    std::stringstream ofss;
    for (int expt_file_ctr = 0; expt_file_ctr < num_robots.size(); ++expt_file_ctr)
    {
      for(int eps_index = 0; eps_index < eps_array.size(); ++eps_index)
      {
          communicationEpsilon = eps_array[eps_index];
        // Read config yaml, based on map id and # of robots.
        multiagent::experiment_config config;
        char fname[80];
        ROS_INFO("Number of Robots %d",num_robots[expt_file_ctr]);
        sprintf(fname, "%s/config/r_%dmap_%d.yaml", base_folder_.c_str(),
                num_robots[expt_file_ctr], 0);
        ROS_INFO("reading %s", fname);
        bool succ = false;
        succ = get_exp_config_header(fname, config);
        if (!succ)
        {
            ROS_ERROR("Error reading config.");
            continue;
        }
        ofss.str(std::string());
        ofss.clear();
        ofss << outputFolder<<"/"<<"commEps"<<communicationEpsilon<<"robots"<<num_robots[expt_file_ctr]<<".csv";
        outputFilename = ofss.str();
        ROS_INFO("saving to file %s",outputFilename.c_str());
        dataDumper.open(outputFilename.c_str(),std::ofstream::out | std::ofstream::trunc);
        dataDumper<<"Total Cost"<<","<<"Communication Events"<<endl;
        // test_nums start at 1, coz matlab.
        for(int test_num = 1; test_num <= config.num_tests; test_num++)
        {
            succ = get_exp_config(fname, test_num, config);
            if (!succ)
            {
                ROS_ERROR("Error reading config.");
                continue;
            }
            char map_fname[80];
            double origin[3];
            origin[0] = 0;
            origin[1] = 0;
            origin[2] = 0;
            sprintf(map_fname, "%s/maps/%s", base_folder_.c_str(), config.map_known.c_str());  //change back to map_known
            ROS_INFO("setting common map from %s", map_fname);
            map_reader::loadMapFromFile(commonmap, map_fname, 0.1, // resolution
                                        false, 0.1, // occupied_threshold
                                        0.01, // free thresh
                                        origin,
                                        true);
            commonmap.data.assign(commonmap.data.size(),0);

            sprintf(map_fname, "%s/maps/%s", base_folder_.c_str(), config.map_unknown.c_str());
            ROS_INFO("setting true map from %s", map_fname);
            map_reader::loadMapFromFile(truemap, map_fname, 0.1, // resolution
                                        false, 0.1, // occupied_threshold
                                        0.01, // free thresh
                                        origin,
                                        true);
            total_robots = total_number_of_robots;
            char mergedMapTopic[80];
            sprintf(mergedMapTopic,"eps%s/mergedMap",testID.c_str());
            commonMapPublisher_ = nh.advertise<nav_msgs::OccupancyGrid>(mergedMapTopic,1000);
            char exptTrueMapTopic[80];
            sprintf(exptTrueMapTopic,"eps%s/exptTrueMap",testID.c_str());
            exptTrueMapPublisher_ = nh.advertise<nav_msgs::OccupancyGrid>(exptTrueMapTopic,1000);

            srand (1);

            for (int i = 0; i < num_starts_per_map; ++i)
            {
                communicationRequired = true;
                set_up_experiment(config);
                ROS_INFO("Successfully setup experiment for run %d on map %d.",i, test_num);
                simulate();
                ROS_INFO("Completed experiment for run %d on map %d.",i, test_num);
                ROS_INFO("Total cost is: %d, Communication events are: %d",total_cost, communication_events); // UNCOMMENT SBPL FUNC
                dataDumper<<total_cost<<","<<communication_events<<endl;
                              // WRITE RESULTS HERE.
            }
            dataDumper.close();
        }
    }
  }

    ROS_INFO("Successfully concluded experiment.");
}

void multiagent::set_up_experiment(const multiagent::experiment_config &config)
{
    std::vector<std::vector<double> > starts;
    std::vector<std::vector<double> > goals;
    std::vector<double> start;
    std::vector<double> goal;

    // Try random starts and goals until a valid experiment.
    // Can speed up by just checking pairs of starts/goals at a time.
  for(int i=0;i<total_robots;i++)
  {
    bool is_valid_experiment = false;
    while (!is_valid_experiment)
    {
        get_random_start_goal_pair(config.map_width,
                                    config.map_height,
                                    config.num_robots,
                                    start,
                                    goal);
        robots_[i].setStart(start);
        robots_[i].setGoal(goal);

        is_valid_experiment = is_exp_valid(i,config,start,goal);




    }
    starts.push_back(start);
    goals.push_back(goal);
  }
  const bool succ = populateRobots(config, starts, goals);
}

bool multiagent::is_exp_valid(int i,const multiagent::experiment_config &config, std::vector<double> start, std::vector<double> goal)
{

            //set all maps to the true map.
            //robots_[i].Initialize(i,start,goal,truemap,truemap,)
            std::vector<sbpl_2Dpt_t> footprint;
            std::string mprimfile;
            mprimfile = config.mprim_filenames[i];
            footprint = config.footprints[i];
            // ROS_INFO("Initializing %d", i);
            // ROS_INFO("start: %f, %f, %f, %f", starts[i][0], starts[i][1], starts[i][2], starts[i][3]);
            // ROS_INFO("goal: %f, %f, %f, %f", goals[i][0], goals[i][1], goals[i][2], goals[i][3]);
            // ROS_INFO("mprimfile: %s", mprimfile.c_str());
            robots_[i].Initialize(i, start, goal, truemap, truemap, mprimfile, footprint,
                                  plannerEpsilon, allocatedTime, stopAfterFirstSolution, backwardSearch);

            //reset all expended costs to 0
            robots_[i].resetExpendedCost();
            const bool succ = robots_[i].updateEnv(robots_[i].getCurrent(),robots_[i].getGoal(), robots_[i].getMap());

            if (!succ)
            {
                //ROS_INFO("update env return false");
                return false;
            }
            int plan_cost = 0;
            const bool plan_succ = robots_[i].makePlan(plan_cost);
            if (!plan_succ)
                return false;

    return true;
}

void multiagent::simulate()
{
  int max_plan_cost = -1;
  total_cost = 0;
  communication_events = 0;
  int count = 0;
  while(max_plan_cost != 0) //Termination condition for robots
  {
    //ROS_INFO("Max Plan Cost is %d",max_plan_cost);
    if(communicationRequired)
    {
      if(count!=0)
      //  ROS_INFO("Communication Event!");
        communication_events ++;
      resetCostsRequired = true;
      for(int i=0; i<total_robots; i++)
      {
        //reset all starts of robots to current positions
        robots_[i].setStart(robots_[i].getCurrent());
        //reset all maps to merged map
        robots_[i].setMap(commonmap,truemap);
        //reset all expended costs to 0
        robots_[i].resetExpendedCost();
        int plan_cost = 0;
        robots_[i].updateEnv(robots_[i].getCurrent(),robots_[i].getGoal(), robots_[i].getMap());
        const bool plan_success = robots_[i].makePlan(plan_cost);
        //ROS_INFO("Plan success is :%d",plan_success);
        //ROS_INFO("setting original cost of robot %d to %d", i, plan_cost);
        robots_[i].setOriginalCost(plan_cost);

      }
      }
      communicationRequired = false;

    max_plan_cost = takeStep();
    mergeMaps();
    commonMapPublisher_.publish(commonmap);
    exptTrueMapPublisher_.publish(truemap);
    count++;
  }
}
int multiagent::takeStep()
{
  int max_plan_cost = 0;
  for(int i=0; i<total_robots; i++)
  {
    int plan_cost = 0;
    for(int j=0;j<total_robots; j++)
      {
        double eval_cost = 0;
        if(i==j)
          {
            //i plans for itself
            //ROS_INFO("robot %d planning for robot %d",i,j);
            eval_cost+=robots_[j].getExpendedCost();
            robots_[j].updateEnv(robots_[j].getCurrent(),robots_[j].getGoal(), robots_[i].getMap());
            const bool plan_succ = robots_[j].makePlan(plan_cost);
            total_cost += robots_[i].advanceRobot();
            //ROS_INFO("Advanced Robot %d",i);
            max_plan_cost = std::max(max_plan_cost,plan_cost);
            //ROS_INFO("Max plan cost is %d",plan_cost);

          }
        else
          {
            //i plans for j
            //ROS_INFO("robot %d planning for robot %d",i,j);
            robots_[j].updateEnv(robots_[j].getStart(),robots_[j].getGoal(), robots_[i].getMap());
            const bool plan_succ = robots_[j].makePlan(plan_cost);
          }

        //ROS_INFO("plan_cost %f",plan_cost);
        eval_cost += plan_cost;
        if(eval_cost > communicationEpsilon*robots_[j].getOriginalCost())
        {
          //ROS_INFO("Communication Epsilon is %f, for robot %d,%d, plan cost is %d, eval cost is %f, original cost is %f, expended_cost %f",
          //         communicationEpsilon, j, i, plan_cost, eval_cost, robots_[j].getOriginalCost(), robots_[j].getExpendedCost());
          communicationRequired = true;
        }
      }
    //ROS_INFO("Plan for robot %d costs %f",i,plan_cost);
    publish(i);
    ros::Duration(0.01).sleep();
  }
  return(max_plan_cost);

}

void multiagent::publish(int i)
{
  currentPosePublisher_[i].publish(robots_[i].getCurrentPose());
  startPosePublisher_[i].publish(robots_[i].getStartPose());
  goalPosePublisher_[i].publish(robots_[i].getGoalPose());
  plannedPathPublisher_[i].publish(robots_[i].getPlannedPath());
  traversedPathPublisher_[i].publish(robots_[i].getTraversedPath());
  mapPublisher_[i].publish(robots_[i].getMap());
  polygonPublisher_[i].publish(robots_[i].getFootprint());
}

bool multiagent::populateRobots(const multiagent::experiment_config &config,
                                const std::vector<std::vector<double> > &starts,
                                const std::vector<std::vector<double> > &goals)
{
  //robots_.clear();
  //robots_.resize(total_robots);
  bool succ = false;
  for(int i=0; i<total_robots; i++)
  {
    std::vector<sbpl_2Dpt_t> footprint;
    std::string mprimfile;

    mprimfile = config.mprim_filenames[i];
    footprint = config.footprints[i];
    // ROS_INFO("Initializing %d", i);
    // ROS_INFO("start: %f, %f, %f, %f", starts[i][0], starts[i][1], starts[i][2], starts[i][3]);
    // ROS_INFO("goal: %f, %f, %f, %f", goals[i][0], goals[i][1], goals[i][2], goals[i][3]);
    // ROS_INFO("mprimfile: %s", mprimfile.c_str());
    commonmap.data.assign(commonmap.data.size(),0);
    robots_[i].Initialize(i, starts[i], goals[i], commonmap, truemap, mprimfile, footprint,
                          plannerEpsilon, allocatedTime, stopAfterFirstSolution, backwardSearch);
    // ROS_INFO("Initialized %d", i);
    //ROS_INFO("Initialized %d", i);


    //robots[i].setMap(map);
  }
  return true;
}
void multiagent::mergeMaps()
{
  for(int i=0; i<total_robots; i++)
  {
    const nav_msgs::OccupancyGrid temp_map = robots_[i].getMap();
    for(int x=0;x<temp_map.data.size(); x++)
    {
      commonmap.data[x] = std::max(commonmap.data[x],temp_map.data[x]);
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "master");
  multiagent multiagent_controller;
  robotClass robot;
  std::vector<int> map_ids;
  map_ids.push_back(0);
  std::vector<int> all_num_robots;
  all_num_robots.push_back(multiagent_controller.total_number_of_robots);
  std::vector<double> eps_array;
  eps_array.push_back(1.0);
  eps_array.push_back(2.0);
  eps_array.push_back(5.0);
  eps_array.push_back(10.0);


  // Run 10 random start/goal pairs for each config.
  const int num_starts_per_map = 10;
  multiagent_controller.start_experiments(num_starts_per_map,
                                          map_ids, all_num_robots,eps_array);
  ROS_INFO("Now Shutting Down");
  ros::shutdown();
}
