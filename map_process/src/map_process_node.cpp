#include "map_process/map_process.h"

using namespace std;

map_process_task::map_process_task(): map_name("null")
{
    signal(SIGINT,signalHandler);
    ros::NodeHandle n,nh("~");
    nh.param<string>("map_file_path",map_file_path,"/home/lxd");
    target_sub = n.subscribe("target_pose_execu",10,&map_process_task::target_publish,this);
    cancel_taregt = n.subscribe("task_execu",1,&map_process_task::cancel_target,this);
    // actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
}

map_process_task::~map_process_task()
{

}

void map_process_task::signalHandler(int signum)
{
  ROS_INFO("%s is received, Terminating the node...",strsignal(signum));
  ros::shutdown();
  exit(signum);
}


void map_process_task::cancel_target(const map_process::task::ConstPtr& msg)
{
    if(msg->task == "cancel_navigation")
        ac.cancelGoal();
    if(msg->task == "load_map")
        map_name = msg->map_name + ".yaml";
    if(msg->task == "save_map")
        save_map(msg->map_name);
    if(msg->task == "delete_map")
        delete_map_file(map_file_path+msg->map_name);
}

void map_process_task::delete_map_file(string delete_map)
{
    string delete_map_yaml(delete_map + ".yaml");
    string delete_map_pgm(delete_map + ".pgm");
    const char *mapPath = delete_map_yaml.c_str();
    const char *map_yaml_Path = delete_map_pgm.c_str();
    if(!remove(mapPath)&&!remove(map_yaml_Path))
        ROS_INFO("delete map successful!!!");
    else 
        ROS_INFO("map is not exit, delete failed!!!");
}

void map_process_task::save_map(string save_map)
{
    std::string mapname = map_file_path + save_map;
    int threshold_occupied = 65;
    int threshold_free = 25;
    MapGenerator mg(mapname, threshold_occupied, threshold_free);
    while(!mg.saved_map_ && ros::ok())
        ros::spinOnce();
}

void map_process_task::static_map(std::string fname,double res)
{
    try
    {
        MapServer ms(fname, res);
        ros::MultiThreadedSpinner s(4);
        ros::spin(s);
        
    }
    catch(std::runtime_error& e)
    {
        ROS_ERROR("map_server exceion: %s", e.what());
    }
}


void map_process_task::target_publish(const map_process::taregt_pose::ConstPtr& msg)
{
    
    if(!ac.waitForServer(ros::Duration(60)))
    {
      ROS_INFO("Can't connected to move base server");
      return ;
    }
    map<string,move_base_msgs::MoveBaseGoal> move_goal;
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = msg->target_pose_x;
    goal.target_pose.pose.position.y = msg->target_pose_y;
    goal.target_pose.pose.orientation =  tf::createQuaternionMsgFromRollPitchYaw(0, 0, msg->target_pose_theta);
    move_goal.insert(pair<string,move_base_msgs::MoveBaseGoal>(msg->map_name,goal));
    ROS_INFO("start_target_move");
    move_goal.begin()->second.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(move_goal.begin()->second);
    ROS_INFO("start move to target %s",move_goal.begin()->first.c_str());
    bool finished_within_time = ac.waitForResult(ros::Duration(180));//执行任务在180s之内,超出后取消任务
    if(!finished_within_time)
    {
        ac.cancelGoal();
        ROS_INFO("Timed out achieving goal");
    }
    else
    {
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("target %s succeeded!",move_goal.begin()->first.c_str());
    }
    else
    {
        ROS_INFO("The base failed for some reason, it can be other task is interrupting");
    }
    }
}

void map_process_task::get_map_file_name(const string& map_path)
{
    DIR * dir;
    string load_map_name;
    struct dirent * ptr;
    dir = opendir(map_path.c_str()); //打开一个目录
    while((ptr = readdir(dir)) != NULL) //循环读取目录数据
    {
        string s(ptr->d_name);
        if(s == "." || s == ".." || s.substr(s.length()-5) == ".pgm")
            continue;
        else
            map_file_name.push_back(s);
    }
    closedir(dir);//关闭目录指针
    if(!map_file_name.empty())
    {
        cout<<"there are "<<map_file_name.size()<<" map, map_name are respectively: ";
        for(auto &map_name : map_file_name)
        {
            cout<<map_name.substr(0,map_name.length()-4)<<",";
        }
        cout<<endl;
        ROS_INFO("please input load map name:");
        while(true)
        {
            cin>>load_map_name;
            load_map_name = load_map_name + ".yaml";
            cout<<load_map_name<<endl;

            auto it = find(map_file_name.begin(), map_file_name.end(), load_map_name);
            if(it!=map_file_name.end())
            {
                load_map_name = map_path + load_map_name;
                static_map(load_map_name,0.0);
            }
            else
            {
                cout<<"map is not exit,please input again"<<endl;
                continue;
            }
        }
    }
}




int main(int argc, char **argv)
{
    ros::init(argc,argv,"map_server");
    map_process_task map_p;
    while(map_p.map_name == "null")
        ros::spinOnce();
    map_p.static_map(map_p.map_file_path+map_p.map_name,0.0);
}
