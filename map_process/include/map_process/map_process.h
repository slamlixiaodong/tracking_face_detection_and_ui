#include "ros/ros.h"
#include "csignal"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "tf/tf.h"
#include <map_process/taregt_pose.h>
#include <map_process/task.h>

#include <iostream>
#include <sys/types.h>
#include <dirent.h> 
#include <unistd.h>
#include <string.h>
#include <cstdio>
#include <vector>
#include <algorithm>


#include "map_process/map_server.h"
#include "map_process/map_saver.h"

using namespace std;

class map_process_task{

public:
    map_process_task();
    ~map_process_task();
    void target_publish(const map_process::taregt_pose::ConstPtr& msg);
    void cancel_target(const map_process::task::ConstPtr& msg);
    void static_map(std::string fname,double res);
    std::string map_name,map_file_path;

private:
    static void signalHandler(int signum);
    void delete_map_file(string delete_map);
    void save_map(string save_map);
    void get_map_file_name(const string& map_path);
    
private:
    std::vector<std::string> map_file_name;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac{"move_base",true};
    ros::Subscriber target_sub,cancel_taregt;
};