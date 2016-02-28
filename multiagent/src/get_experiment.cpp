#include <multiagent/multiagent.h>
#include <stdlib.h>

namespace utils
{
std::vector<sbpl_2Dpt_t> get_footprint(const int robot_id)
{
    // TODO: hardcoded for now.
    std::vector<sbpl_2Dpt_t> footprint(5);
    footprint[0].x = -0.325;
    footprint[0].y = -0.425;
    footprint[1].x = -0.325;
    footprint[1].y = 0.425;
    footprint[2].x = 0.325;
    footprint[2].y = 0.425;
    footprint[3].x = 0.46;
    footprint[3].y = 0.0;
    footprint[4].x = 0.325;
    footprint[4].y = -0.425;
    return footprint;
}

std::string get_primitive_file(const int robot_id, const std::string &base_path)
{
    switch(robot_id)
    {
    case 1: return base_path + std::string("/config/robot1.mprim");
        break;
    case 2: return base_path + std::string("/config/robot2.mprim");
        break;
    }
}
}

bool multiagent::get_exp_config_header(const char* filename,
                                       multiagent::experiment_config &config) const
{
    config.mprim_filenames.clear();
    FILE* fin = fopen(filename,"r");
    if(!fin){
        ROS_ERROR("file %s does not exist\n", filename);
        return false;
    }
    // Motion primitive filenames.
    std::vector<std::string> mprim_filenames;
    fscanf(fin,"experiments:\n\n");
    fscanf(fin, "  num_robots: %d\n", &config.num_robots);    
    config.mprim_filenames.clear();
    config.footprints.clear();
    // Read motion primitive files.
    for (int i = 0; i < config.num_robots; ++i){
        int robot_id;
        if(fscanf(fin, "  robot_id: %d", &robot_id) <= 0)
            { ROS_INFO("ID failed");
            return false;        
    }
        config.mprim_filenames.push_back(utils::get_primitive_file(robot_id, base_folder_));
        config.footprints.push_back(utils::get_footprint(robot_id));
    }

    if(fscanf(fin, "  num_tests: %d\n", &config.num_tests) <= 0)
        return false;
    return true;
}

bool multiagent::get_exp_config(const char* filename,                                
                                const int desired_test_number,
                                multiagent::experiment_config &config) const
{
    ROS_INFO("Reading header with test # %d", desired_test_number);
    config.mprim_filenames.clear();
    FILE* fin = fopen(filename,"r");
    if(!fin){
        ROS_ERROR("file %s does not exist\n", filename);
        return false;
    }
    // Motion primitive filenames.
    std::vector<std::string> mprim_filenames;
    fscanf(fin,"experiments:\n\n");
    fscanf(fin, "  num_robots: %d\n", &config.num_robots);
    config.mprim_filenames.clear();
    config.footprints.clear();
    // Read motion primitive files.
    for (int i = 0; i < config.num_robots; ++i){
        int robot_id;
        if(fscanf(fin, "  robot_id: %d", &robot_id) <= 0)
            return false;        
        config.mprim_filenames.push_back(utils::get_primitive_file(robot_id, base_folder_));
        config.footprints.push_back(utils::get_footprint(robot_id));
    }
    if(fscanf(fin, "  num_tests: %d", &config.num_tests) <= 0)
        return false;
    int test_num = 0;
    while(fin)
    {
        if(fscanf(fin,"  - test: %d\n", &test_num) <= 0)
            return false;
        char map_known[80];
        char map_unknown[80];
        if (fscanf(fin,"    map_known: %s", &map_known) <= 0)
            return false;
        if (fscanf(fin,"    map_unknown: %s", &map_unknown) <= 0)
            return false;
        config.map_known = std::string(map_known);
        config.map_unknown = std::string(map_unknown);
        config.map_width = 30;
        config.map_height = 30;
        if (test_num == desired_test_number)
        {
            return true;
        }
    }
    return false;
}

void multiagent::get_random_start_goal_pairs(const int map_width,
                                             const int map_height,
                                             const int num_robots,
                                             std::vector<std::vector<double> > &starts,
                                             std::vector<std::vector<double> > &goals) const
{    
    //ROS_INFO("in get_random_start_goal map height: %d width: %d", map_height, map_width);
    starts.clear();
    goals.clear();
    for (int i = 0; i < num_robots; i++){
        std::vector<double> start(4, 0);
        std::vector<double> goal(4, 0);        
        const double start_theta = (2*M_PI/10) * (double)(std::rand()%10);
        const double goal_theta = (2*M_PI/10) * (double)(std::rand()%10);
        
        start[0] = 0.5 + (std::rand() % map_width); // x
        start[1] = 0.5 + (std::rand() % map_height); // y
        start[2] = 0; // z
        start[3] = start_theta;        
        starts.push_back(start);
        goal[0] = 0.5 + (std::rand() % map_width); // x
        goal[1] = 0.5 + (std::rand() % map_height); // y
        goal[2] = 0; // z
        goal[3] = goal_theta;    
        goals.push_back(goal);        
        //ROS_INFO("Robot %d start: %2.2f, %2.2f, %2.2f, %2.2f", i, start[0], start[1], start[2], start[3]);
        //ROS_INFO("Robot %d goal: %2.2f, %2.2f, %2.2f, %2.2f", i, goal[0], goal[1], goal[2], goal[3]);
    }
}

