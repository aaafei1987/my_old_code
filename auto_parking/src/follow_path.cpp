/*
 * @Author: hitflx
 * @Date: 2021-09-02 15:40:50
 * @LastEditTime: 2022-01-12 17:03:28
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /relocation_with_position/src/ndt_first_match.cpp
 */
#include "follow_path.h"



FollowPath::FollowPath()
{
    next_pose_index_ = -1;
    have_config_ = false;
    have_waypoints_ = false;
    have_current_pose_ = false;
    have_current_ori_ = false;
    new_path_flag_ = false;

    line_vel_ = 0.1;
    line_vel_yaml_ = 0.3;
    lookahead_min_distance_ = 0.5;
    lookahead_max_distance_ = 3.0;
    limit_max_angular_vel_ = 0.2;
    limit_min_angular_vel_ = 0.01;
    min_trunning_radius_ = 2.0;
    max_trunning_radius_ = 900000000; //走直线情况下近似回转无穷大，这个参数不会放在config里
    start_dis_thread_ = 2.0;

    tolarte_ori_ = PI/6.0;
    is_pause_ = false;
}

bool FollowPath::InitConfig(const YAML::Node yaml_node)
{
    lookahead_min_distance_ = yaml_node["lookahead_min_distance"].as<float>();
    lookahead_max_distance_ = yaml_node["lookahead_max_distance"].as<float>();
    if(lookahead_max_distance_ < lookahead_min_distance_) return false;
    line_vel_yaml_ = yaml_node["line_vel"].as<float>();
    line_vel_ = line_vel_yaml_;
    limit_min_angular_vel_ = yaml_node["limit_min_angular_vel"].as<float>();
    limit_max_angular_vel_ = yaml_node["limit_max_angular_vel"].as<float>();
    tolarte_ori_ = yaml_node["tolarte_ori"].as<float>();
    min_trunning_radius_ = yaml_node["min_trunning_radius"].as<float>();
    start_dis_thread_ = yaml_node["start_dis_thread"].as<float>();
    distance_to_goal_ = yaml_node["distance_to_goal"].as<float>();
    distance_slow_to_goal_ = yaml_node["distance_slow_to_goal"].as<float>();
    vel_to_goal_ = yaml_node["vel_to_goal"].as<float>();
    have_config_ = true;
    return true;
}


bool FollowPath::GetNextWaypoints()
{
    //初始化
    if(next_pose_index_<0)
    {
        double start_dis = SqrtDistance(waypoints_[0], current_pose_);
        if(start_dis > start_dis_thread_)
        {
            LOG(ERROR)<<"start pose is so far to waypoints first";
            return false;
        }
        
        ROS_INFO("next waypoint is 0");
        next_pose_index_ = 0;
        current_index_ = 0; //当前也从0开始
        //计算前进还是倒退
        double path_ori = atan2(waypoints_[2].second-waypoints_[0].second , waypoints_[2].first-waypoints_[0].first);
        LOG(INFO)<<"path ori "<<path_ori <<" cur ori " << current_ori_;
        double delta = RoudingRad(path_ori - current_ori_);
        if(abs(delta) < PI/3.0)
        {
            //ROS_INFO("forward mode . delta is %d" , delta);
            LOG(INFO)<<"forward mode ,delta : " <<delta ;
            forward_ = true;
        }
        else if(abs(delta) > 2*PI/3)
        {
            //ROS_INFO("backward mode . delta is %d" , delta);
            LOG(INFO)<<"backward mode ,delta : " <<delta ;
            forward_ = false;            
        }
        else
        {
            //ROS_ERROR("wrong start ori . delta is %d" , delta);
            return false;              
        }

        return true;
    }


    for(int i=next_pose_index_ ; i<waypoints_.size() ; i++)
    {
        if(i==(waypoints_.size()-1))
        {
            //no need to log it
            ROS_INFO("next waypoint is final");
            next_pose_index_ = i;
            return true;
        }

        double dis = SqrtDistance(waypoints_[i], current_pose_);
        
        if((dis >= lookahead_min_distance_) &&(dis <= lookahead_max_distance_))
        {
            //no need to log it
            ROS_INFO("next waypoint is %d" , i);
            next_pose_index_ = i;
            return true;
        }
    }

    LOG(ERROR)<<"no next index";
    return false;
}

bool FollowPath::GetVelocity(Velocity2d &cmd_vel)
{
    if(!have_config_ || !have_current_pose_ || !have_current_ori_ || !have_waypoints_)
    {
        //ROS_INFO("condition is not satisfied ");
        return false;
    }
    
    //暂停
    if(is_pause_)
    {
        LOG(INFO) <<"pause , stop car";
        cmd_vel.first = 0.0;
        cmd_vel.second = 0.0;
        return true;
    }

    if(!GetNextWaypoints()) return false;
    //如果到达了终点 则停下
    if(next_pose_index_ == waypoints_.size()-1) 
    {
        if(CheckFinished())
        {
            cmd_vel.first = 0.0;
            cmd_vel.second = 0.0;
            //ROS_INFO(" reach the final waypoint , reset State ");
            return true;
        }
    }

    //安全判断 如果当前位置偏离轨迹 ， 则停车
    if(!CheckSecurity())
    {
        LOG(ERROR) <<"curpose lost in waypoints";
        cmd_vel.first = 0.0;
        cmd_vel.second = 0.0;
        return false;
    }

    if(forward_)
        cmd_vel = computeVelocity(computeCurvature(waypoints_[next_pose_index_]), true);
    else  //角度相差pi 倒着走
        cmd_vel = computeVelocity(computeCurvature(waypoints_[next_pose_index_]), false);


    return true;
}

bool FollowPath::PauseParking()
{
    LOG(INFO) << "Pause parking or outing";
    is_pause_ = true;
    return true;
}

bool FollowPath::ContinueParking()
{
    LOG(INFO) << "Cancel Pause parking or outing";
    is_pause_ = false;
    return true;
}

bool FollowPath::CheckSecurity()
{
    if(next_pose_index_<=0)return true;
    //计算当前点到轨迹上前后点的最近距离，大于一定值就返回false
    double min_dis = 100;
    for(int i=0 ; i<next_pose_index_ ; i++)
    {
        double dis = SqrtDistance(current_pose_ , waypoints_[i]);
        if(dis < min_dis) min_dis = dis;
    }
    
    if(min_dis < 0.5) return true;
    else
        return false;
}

void FollowPath::ResetState()
{
    //速度还原
    line_vel_ = line_vel_yaml_;
    next_pose_index_ = -1;
    have_current_pose_ = false;
    have_current_ori_ = false;
    have_waypoints_ = false;
    new_path_flag_ = false;
}

bool FollowPath::CheckFinished()
{
    if(!have_waypoints_)
        return false;
    //走到倒数第二个点 最后一个点是过调的
    int end_index = waypoints_.size()-2;
    if(SqrtDistance(waypoints_[end_index] , current_pose_) < distance_slow_to_goal_)
    {
        //降速 若到终点返回true
        line_vel_ = vel_to_goal_;
        if(SqrtDistance(waypoints_[end_index] , current_pose_) < distance_to_goal_)
            return true;
        else
            return false;       
    }

    return false;
}

Velocity2d FollowPath::computeVelocity(double kappa , bool forward)
{
    if(forward)
    {
       
        //向前走 常规操作
        Velocity2d cmd_vel;
        cmd_vel.first = line_vel_;
        cmd_vel.second = kappa*line_vel_;
        //LOG(INFO) <<"forward cmd x: "<<cmd_vel.first <<" y" << cmd_vel.second <<" 1/r: "<< kappa;

        return cmd_vel;
    }
    else
    {
        //倒着走
        Velocity2d cmd_vel;
        cmd_vel.first = -line_vel_;
        cmd_vel.second = -kappa*line_vel_; //?存疑 实际测试看是否还需要修改
        //LOG(INFO) <<"back cmd x: "<<cmd_vel.first <<" y" << cmd_vel.second <<" 1/r: "<< kappa;
    
        return cmd_vel;        
    }
}

double FollowPath::computeCurvature(Pose2d next_pose)
{
    double kappa;
    //LOG(INFO)<<"cal kappa, next pose x: "<<next_pose.first <<" y: " <<next_pose.second ;
    //LOG(INFO)<<"curr pose x " <<current_pose_.first <<" y: " <<current_pose_.second ;
    double denominator = pow((next_pose.second - current_pose_.second), 2) + pow((next_pose.first - current_pose_.first) , 2);
    double numerator = 2 * calcRelativeCoordinate(next_pose, current_pose_ , current_ori_).y;
    //LOG(INFO)<<"denominator : "<<denominator <<" numerator: "<<numerator;
    //r = ld / 2sinα 这里是曲率 取倒数
    if (denominator != 0)
    {
        //最小回转半径要求
        kappa = numerator / denominator;
        //LOG(INFO)<< "kappa is "<<kappa;
        double turn_r = 1.0/kappa;
        if(turn_r >0 && turn_r < min_trunning_radius_)
            kappa = 1.0/min_trunning_radius_;
        else if(turn_r < 0 && turn_r > -min_trunning_radius_)
            kappa = - 1.0/min_trunning_radius_;
    }
    else
    {
        //就是走直线
        if (numerator > 0)
            kappa = 1.0/max_trunning_radius_;
        else
            kappa = -1.0/max_trunning_radius_;
    }
    return kappa;
}

//next point 在 curpose 坐标系下看到的位置（相对坐标）
geometry_msgs::Point FollowPath::calcRelativeCoordinate(Pose2d next_pose, Pose2d current_pose , double current_ori)
{
    //利用tf工具转换 当前坐标系
    tf::Transform cur;
    tf::Vector3 origin(current_pose.first , current_pose.second , 0);
    cur.setOrigin(origin);
    tf::Quaternion ori;
    ori.setRPY(0.0 , 0.0 , current_ori);
    cur.setRotation(ori);

    tf::Point target = tf::Vector3(next_pose.first , next_pose.second , 0.0);
    tf::Point transformed  = cur.inverse() * target;
    geometry_msgs::Point point_return;
    tf::pointTFToMsg(transformed, point_return);
    //LOG(INFO)<<"get trans point x: "<<point_return.x <<" y: "<<point_return.y <<" z: " << point_return.z ;
    return point_return;
 
}