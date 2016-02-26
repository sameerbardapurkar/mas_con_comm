#include <multiagent/multiagent.h>

bool multiagent::get_exp_config(const char* filename,
                    experiment_config &config)
{
    config.mprim_filenames.clear();
    config.start_x.clear();
    config.start_y.clear();
    config.start_z.clear();
    config.start_theta.clear();
    config.goal_x.clear();
    config.goal_y.clear();
    config.goal_z.clear();
    config.goal_theta.clear();

    FILE* fin = fopen(filename,"r");
    if(!fin){
        ROS_ERROR("file %s does not exist\n", filename);
        return false;
    }
    // Motion primitive filenames.
    std::vector<std::string> mprim_filenames;
    fscanf(fin,"experiments:\n\n");
    fscanf(fin, "  num_robots: %d\n", &config.num_robots);
    // Read motion primitive files.
    for (int i = 0; i < config.num_robots; ++i){
        char mprim_filename[80];
        if(fscanf(fin, "  primitive_file: %s", &mprim_filename) <= 0)
            break;
        config.mprim_filenames.push_back(std::string(mprim_filename));
    }
    int test_num = 0;
    if(fscanf(fin,"  - test: %d\n", &test_num) <= 0)
        return false;
    for (int i = 0; i < config.num_robots; ++i){
        double start_x, start_y, start_z, start_theta;
        double goal_x, goal_y, goal_z, goal_theta;
        if(fscanf(fin,"    start: %lf %lf %lf %lf\n", &start_x, &start_y, &start_z, &start_theta) <= 0)
            return false;
        if(fscanf(fin,"    goal: %lf %lf %lf %lf\n", &goal_x, &goal_y, &goal_z, &goal_theta) <= 0)
            return false;
        config.start_x.push_back(start_x);
        config.start_y.push_back(start_y);
        config.start_z.push_back(start_z);
        config.start_theta.push_back(start_theta);
        config.goal_x.push_back(goal_x);
        config.goal_y.push_back(goal_y);
        config.goal_z.push_back(goal_z);
        config.goal_theta.push_back(goal_theta);
    }
    // Hardcoded footprint.
    config.footprints.resize(config.num_robots);
    for (int i = 0; i < config.num_robots; ++i)
        {
            config.footprints[i] = get_footprint();
        }
    return true;
}

std::vector<sbpl_2Dpt_t> multiagent::get_footprint()
{
    // TODO: hardcoded for now.
    std::vector<sbpl_2Dpt_t> footprint(5);
    footprint[0].x = -0.0325;
    footprint[0].y = -0.0425;
    footprint[1].x = -0.0325;
    footprint[1].y = 0.0425;
    footprint[2].x = 0.0325;
    footprint[2].y = 0.0425;
    footprint[3].x = 0.046;
    footprint[3].y = 0.0;
    footprint[4].x = 0.0325;
    footprint[4].y = -0.0425;
    return footprint;
}

void multiagent::print_exp_config(const experiment_config &config)
{
    for(int i = 0; i < config.num_robots; ++i){
        ROS_INFO("Robot %d start = (%f, %f, %f, %f) "
                 "goal = (%f, %f, %f, %f) ", i,
                 config.start_x[i], config.start_y[i], config.start_z[i], config.start_theta[i],
                 config.goal_x[i], config.goal_y[i], config.goal_z[i], config.goal_theta[i]);
      }
}
