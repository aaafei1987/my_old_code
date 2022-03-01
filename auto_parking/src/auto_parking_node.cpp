/*
 * @Author: your name
 * @Date: 2021-09-30 08:59:30
 * @LastEditTime: 2022-01-12 16:07:58
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /auto_parking/src/auto_parking_node.cpp
 */


#include "generate_path.h"
#include "follow_path.h"
#include "data_utility.h"
#include "src_path_defination.h"



Pose2d cur_ndt_pose_;
double cur_ndt_ori_;
std::vector<Pose2d> caledWaypoints_;
bool get_new_pose_; //获取当前位置
bool finish_path_; //完成path跟踪
bool get_new_path_cmd_; //获取新cmd进行轨迹跟踪
bool obs_flag_; //触发避障

//制作轨迹的参数块
struct {
    bool parking_or_out; //倒车还是出车
    double line_distance; //到达终点前走一段直线的距离
    Pose2d start_xy;
    double start_ori;
    Pose2d end_xy;
    double end_ori;
}creat_param;

ParkStatus last_move_state;
ParkStatus current_move_state;
ParkStatus change_set;


using namespace Eigen;

void ChangeParkStatus(ParkStatus tochange)
{
    if(tochange == current_move_state) return;
    last_move_state = current_move_state;
    current_move_state = tochange;
    LOG(INFO)<<"robot status change from: "<<last_move_state <<"--->"<<current_move_state;
}

void CurPoseCallback(geometry_msgs::PoseStamped ndt_pose)
{
    //ROS_INFO("recv cur pose");
    cur_ndt_pose_.first = ndt_pose.pose.position.x;
    cur_ndt_pose_.second = ndt_pose.pose.position.y;
    tf::Quaternion temp;
    tf::quaternionMsgToTF(ndt_pose.pose.orientation , temp);
    cur_ndt_ori_ = tf::getYaw(temp);
    get_new_pose_ = true;
}

void OdomCallback(nav_msgs::Odometry odom_pose)
{
    cur_ndt_pose_.first = odom_pose.pose.pose.position.x;
    cur_ndt_pose_.second = odom_pose.pose.pose.position.y;
    tf::Quaternion temp;
    tf::quaternionMsgToTF(odom_pose.pose.pose.orientation , temp);
    cur_ndt_ori_ = tf::getYaw(temp);
    get_new_pose_ = true;    
}
//订阅该话题 制造轨迹队列 
void AutoParkSetCallback(auto_parking::SetAutoPark setting_config)
{
    LOG(INFO)<<"recv set cmd";
    LOG(INFO)<<"parking_or_out: "<<setting_config.parking_or_out;
    LOG(INFO)<<"line_distance: "<<setting_config.line_distance;
    LOG(INFO)<<"start_x,y,ori: ["<<setting_config.start_x<<","<<setting_config.start_y<<","<<setting_config.start_ori<<"]";
    LOG(INFO)<<"end_x,y,ori: ["<<setting_config.end_x<<","<<setting_config.end_y<<","<<setting_config.end_ori<<"]";

    creat_param.parking_or_out = setting_config.parking_or_out;
    creat_param.line_distance = setting_config.line_distance;
    creat_param.start_xy.first = setting_config.start_x;
    creat_param.start_xy.second = setting_config.start_y;
    creat_param.start_ori = setting_config.start_ori;
    creat_param.end_xy.first = setting_config.end_x;
    creat_param.end_xy.second = setting_config.end_y;
    creat_param.end_ori = setting_config.end_ori;
    get_new_path_cmd_ = true;
}


void AutoParkResetCallback(std_msgs::Float32 apa_set)
{
     //@Todo
    finish_path_ = true;
}

void AutoParkObstacleCallback(std_msgs::Int8 apa_obs)
{        
    if(apa_obs.data == 1)
        obs_flag_ = true;
    if(apa_obs.data == 0)
        obs_flag_  = false;  
}



int main(int argc, char **argv)
{
    //log
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/log/auto_park_log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "auto_parking_node");
    ros::NodeHandle nh;

    //配置文件路径 
    std::string config_file_path = WORK_SPACE_PATH + "/config/config.yaml";
    //std::cout<<"Load test node config path: "<<config_file_path<<std::endl;
    LOG(INFO)<<"init node and config, node name is: auto_parking_node , config path is: " <<config_file_path;
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    obs_flag_ = false;
    get_new_pose_ = false;
    get_new_path_cmd_ = false;  // 发送cmd进行路径跟踪 只有在finish path = true下才会开始
    finish_path_ = true; //这个为true 表示可以接收新来的path ，收到新path后改为false，直到自己重置或者完成当前plan后才为true
    last_move_state = ParkStatus::NO_MOVE;
    current_move_state = ParkStatus::NO_MOVE;//初始状态为为运动

    std::string localization_topic = "ndt_pose";
	nh.getParam("localization_topic", localization_topic);
    ros::Subscriber sub_current_pose = nh.subscribe(localization_topic , 3, CurPoseCallback);
    //ros::Subscriber sub_odom_pose = nh.subscribe("/odom" , 3, OdomCallback);
    ros::Publisher pub_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel" , 10);
    ros::Publisher pub_status = nh.advertise<auto_parking::AutoParkStatus>("auto_park_status", 10);

    ros::Subscriber sub_apa_cmd = nh.subscribe("auto_park_set" , 1 , AutoParkSetCallback);

    ros::Subscriber sub_reset_cmd = nh.subscribe("auto_park_reset" , 1 ,AutoParkResetCallback);

    ros::Subscriber sub_obstacle_cmd = nh.subscribe("auto_park_obstacle" , 1 ,AutoParkObstacleCallback);

    CreatPath creat_path_class;
    //ROS_INFO("load creat path config");
    if(!creat_path_class.InitConfig(config_node))
    {
        LOG(ERROR) <<"wrong config in path creat";
        //ROS_ERROR("wrong config in path creat");
        return 0;
    }

    FollowPath follow_path_class;
    //ROS_INFO("load follow path config");
    if(!follow_path_class.InitConfig(config_node))
    {
        LOG(ERROR) <<"wrong config in follow path";
        //ROS_ERROR("wrong config in follow path");
        return 0;
    }

    ros::Rate rate_(100);
    int hz_ = 100;
    while(ros::ok()){
        hz_--;
        if(hz_<=0){
            int now_status = (int)current_move_state;
            auto_parking::AutoParkStatus need_to_pub;
            need_to_pub.auto_park_status = now_status;
            pub_status.publish(need_to_pub);
            //LOG(INFO)<<"current robot status is: "<<now_status;
            hz_ = 100;
        }

        if(!get_new_pose_){
            //没有新的当前pose进来就等着
            rate_.sleep();
            ros::spinOnce();
            continue;
        }

        //触发避障就暂停(暂时未实现)
        if(obs_flag_){
            //先停下
            geometry_msgs::Twist pub_cmd;
            pub_cmd.linear.x = 0.0;
            pub_cmd.linear.y = 0.0;
            pub_cmd.linear.z = 0.0;
            pub_cmd.angular.z = 0.0;
            pub_cmd.angular.x = 0.0;
            pub_cmd.angular.y = 0.0;
            pub_vel.publish(pub_cmd);
            //暂停
            if((current_move_state == ParkStatus::FORWARD_MOVE)||(current_move_state == ParkStatus::BACKWARD_MOVE))
                follow_path_class.PauseParking();
            //转换状态 
            if(current_move_state == ParkStatus::FORWARD_MOVE)
                ChangeParkStatus(ParkStatus::FORWARD_PAUSE);
            if(current_move_state == ParkStatus::BACKWARD_MOVE)
                ChangeParkStatus(ParkStatus::BACKWARD_PAUSE);

            rate_.sleep();
            ros::spinOnce();
            continue;           
        }
        else{
            //状态为暂停的话就转换回来
            if((current_move_state == ParkStatus::FORWARD_PAUSE))
            {
                follow_path_class.ContinueParking();
                ChangeParkStatus(ParkStatus::FORWARD_MOVE);
            }

            if((current_move_state == ParkStatus::BACKWARD_PAUSE))
            {
                follow_path_class.ContinueParking();
                ChangeParkStatus(ParkStatus::BACKWARD_MOVE);
            }
        }

        if(finish_path_)
        {
            //说明完成path 并有新的cmd进来开始执行
            //轨迹跟踪的初始化从这里执行
            if(get_new_path_cmd_)
            {
                LOG(INFO)<<"cal new path init";
                if(creat_param.parking_or_out){
                    //start check
                    if(SqrtDistance(cur_ndt_pose_ ,creat_param.start_xy )>1.0)
                    {
                        LOG(ERROR)<<"start pose in cmd is far away to now pose ";
                        LOG(ERROR)<<"start pose in cmd: "<<creat_param.start_xy.first <<"  "<<creat_param.start_xy.second;
                        LOG(ERROR)<<"now pose: "<<cur_ndt_pose_.first <<"  "<<cur_ndt_pose_.second;   
                        get_new_path_cmd_ = false;                     
                    }

                    if(abs(RoudingRad(creat_param.start_ori - cur_ndt_ori_)) > PI/6.0)
                    {
                        LOG(ERROR)<<"start ori in cmd is far away to now pose ori ";
                        LOG(ERROR)<<"start ori in cmd: " << creat_param.start_ori;
                        LOG(ERROR)<<"now ori:"<<cur_ndt_ori_;
                        get_new_path_cmd_ = false; 
                    }
                    //当前位置替换
                    creat_param.start_xy = cur_ndt_pose_;
                    creat_param.start_ori = cur_ndt_ori_;

                }
                else{
                    //start check
                    if(SqrtDistance(cur_ndt_pose_ ,creat_param.end_xy )>1.0)
                    {
                        LOG(ERROR)<<"start pose in cmd is far away to now pose ";
                        LOG(ERROR)<<"start pose in cmd: "<<creat_param.end_xy.first <<"  "<<creat_param.end_xy.second;
                        LOG(ERROR)<<"now pose: "<<cur_ndt_pose_.first <<"  "<<cur_ndt_pose_.second;   
                        get_new_path_cmd_ = false;                     
                    }

                    if(abs(RoudingRad(creat_param.end_ori - cur_ndt_ori_)) > PI/6.0)
                    {
                        LOG(ERROR)<<"start ori in cmd is far away to now pose ori ";
                        LOG(ERROR)<<"start ori in cmd: " << creat_param.end_ori;
                        LOG(ERROR)<<"now ori:"<<cur_ndt_ori_;
                        get_new_path_cmd_ = false; 
                    }
                    //当前位置替换
                    creat_param.end_xy = cur_ndt_pose_;
                    creat_param.end_ori = cur_ndt_ori_;
                }

                if(creat_path_class.CalPath(creat_param.start_xy, creat_param.start_ori , creat_param.end_xy,
                                                creat_param.end_ori , creat_param.line_distance) && get_new_path_cmd_)
                {
                    LOG(INFO)<<"finish new path";
                    
                    if(creat_param.parking_or_out)
                        creat_path_class.GetParkingPath(caledWaypoints_);
                    else
                        creat_path_class.GetOutingPath(caledWaypoints_);
                    
                    //debug  log waypoints
                    LOG(INFO)<<"caled path is :";
                    for(int i=0 ; i<caledWaypoints_.size() ; i++)
                    {
                        //std::cout <<"[ " << caledWaypoints_[i].first <<" , " <<caledWaypoints_[i].second <<" ]" <<std::endl;
                        LOG(INFO)<<"[" << caledWaypoints_[i].first <<" , " <<caledWaypoints_[i].second <<"]";
                    }

                    if(follow_path_class.SetWaypoints(caledWaypoints_))
                    {
                        //完成导入轨迹 
                        follow_path_class.SetCarOrientation(cur_ndt_ori_);
                        follow_path_class.SetCurrentPose(cur_ndt_pose_);
                        LOG(INFO)<<"begin this path";
                        //ROS_INFO("begin this path");
                        finish_path_ = false;
                        get_new_path_cmd_ = false;
                    }
                    else
                    {
                        LOG(ERROR)<<"waypoints error";
                        //ROS_ERROR("waypoints error");
                    }
                }
                else
                {
                    LOG(ERROR)<<"cmd is error , caled path wrong";
                    //ROS_ERROR("cmd is error");
                }
            }
        }
        else{
            //LOG(INFO)<<"running path";
            //此时finish path= false 说明轨迹规划中 要计算出cmdvel 并发布出去
            follow_path_class.SetCarOrientation(cur_ndt_ori_);
            follow_path_class.SetCurrentPose(cur_ndt_pose_);
            Velocity2d cmd_vel;
            if(follow_path_class.GetVelocity(cmd_vel))
            {
                geometry_msgs::Twist pub_cmd;
                pub_cmd.linear.x = cmd_vel.first;
                pub_cmd.linear.y = 0.0;
                pub_cmd.linear.z = 0.0;
                pub_cmd.angular.z = cmd_vel.second;
                pub_cmd.angular.x = 0.0;
                pub_cmd.angular.y = 0.0;
                pub_vel.publish(pub_cmd);
                //LOG(INFO)<<"pub cmd_vel";
                //更改状态
                if(cmd_vel.first >= 0)
                    ChangeParkStatus(ParkStatus::FORWARD_MOVE);
                else
                    ChangeParkStatus(ParkStatus::BACKWARD_MOVE);
            }
            else{
                LOG(ERROR)<<"get vel error , stop robot";
                //ROS_ERROR("get vel error , stop robot");
                geometry_msgs::Twist pub_cmd;
                pub_cmd.linear.x = cmd_vel.first;
                pub_cmd.linear.y = 0.0;
                pub_cmd.linear.z = 0.0;
                pub_cmd.angular.z = cmd_vel.second;
                pub_cmd.angular.x = 0.0;
                pub_cmd.angular.y = 0.0;
                pub_vel.publish(pub_cmd);
                LOG(ERROR)<<"stop";
                finish_path_ = true;
                follow_path_class.ResetState();
                creat_path_class.ResetPathData(); 
                ChangeParkStatus(ParkStatus::STOP);               
            }
        }

        if(follow_path_class.CheckFinished())
        {
            LOG(INFO)<<"finish this waypoints";
            finish_path_ = true;
            follow_path_class.ResetState();
            creat_path_class.ResetPathData();
            ChangeParkStatus(ParkStatus::NO_MOVE);
        }
            
        get_new_pose_ = false;
        rate_.sleep();
        ros::spinOnce();

    }    
    return 0;
    

}


