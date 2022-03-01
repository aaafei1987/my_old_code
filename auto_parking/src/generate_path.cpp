/*
 * @Author: your name
 * @Date: 2021-09-30 08:59:30
 * @LastEditTime: 2021-11-02 17:03:23
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /auto_parking/src/generate_path.cpp
 */
#include "generate_path.h"

CreatPath::CreatPath()
{
    parking_path_.clear();
    outing_path_.clear();
    min_corner_r = -1.0;
    begin_line_distance = 1.0;
    overshoot_line = 2.0;
}

bool CreatPath::InitConfig(const YAML::Node yaml_node)
{
    min_corner_r = (double) yaml_node["min_corner_r"].as<float>();
    begin_line_distance = yaml_node["begin_line_distance"].as<float>();
    overshoot_line = yaml_node["overshoot_line"].as<float>();
    if(min_corner_r<= 0)
    {
 
        min_corner_r = -1;
        ROS_WARN("min_corner  r < 0!");
        return false;
    }

    LOG(INFO)<<"inited config ";
    return true;
    
}

//给定起点与终点，画出行驶轨迹
//垂直倒车 画个圆弧 然后走个直线到达目标点
//先假定start ori未知，最后画出轨迹后比较起始角度 偏差过大就返回false  回转过小也会返回false(todo)
// 路线为直线->圆弧->直线   具体点： start ->circle_start_pose ->circle_end_pose -> end
bool CreatPath::CalPath(Pose2d start , double start_ori , Pose2d end , double end_ori , double line_distance)
{
    if((start.first == end.first)&&(start.second == end.second)) return false;

    //弧线的起点(由于是倒车，所以是减号)
    Pose2d circle_start_pose;
    circle_start_pose.first = start.first - begin_line_distance*cos(start_ori);
    circle_start_pose.second = start.second - begin_line_distance*sin(start_ori);
    //过调 起点额外直线 这段路不会走
    overshoot_begin_pose_.first = start.first + overshoot_line*cos(start_ori);
    overshoot_begin_pose_.second = start.second + overshoot_line*sin(start_ori);
    //弧线的终点 
    Pose2d circle_end_pose;
    circle_end_pose.first = end.first + line_distance*cos(end_ori);
    circle_end_pose.second = end.second + line_distance*sin(end_ori);
    //终点过调
    overshoot_end_pose_.first = end.first - overshoot_line*cos(end_ori);
    overshoot_end_pose_.second = end.second - overshoot_line*sin(end_ori);


    LOG(INFO) <<"circle_start_pose " <<circle_start_pose.first <<" "<<circle_start_pose.second ;
    LOG(INFO) <<"circle_end_pose " <<circle_end_pose.first <<" "<<circle_end_pose.second;

    //计算圆弧圆心（起点与终点连线的中垂线，与终点圆心方向的角点）
    //计算终点方向的垂线斜率 直线方程为 y- circle_end_pose.second = start_k*(x - circle_end_pose.first)
    double start_k = 100000.0; //近似无穷大
    if(abs(end_ori) >= 0.02) start_k = tan(end_ori + PI/2.0);
    //计算中垂线 斜率为end_k =  -start.second - circle_end_pose.second)/(start.first - circle_end_pose.first) 再 -1/k
    // 过中点 start.first + circle_end_pose.first /2， start.second + circle_end_pose.second /2）
    double end_k = 0.0;
    if(abs(circle_start_pose.second - circle_end_pose.second)<0.005) end_k = 10000.0;
    else
        end_k = -(circle_start_pose.first - circle_end_pose.first)/(circle_start_pose.second - circle_end_pose.second);
    
    double end_x = (circle_start_pose.first + circle_end_pose.first)/2.0;
    double end_y = (circle_start_pose.second + circle_end_pose.second)/2.0;
    LOG(INFO)<<"middle line -> k: "<<end_k <<" middle_x : "<<end_x << " middle_y : "<<end_y<<std::endl;
    LOG(INFO)<<"other line -> k: "<<start_k <<" st_x : "<<circle_end_pose.first << " st_y : "<<circle_end_pose.second;
    //计算直线交点 就是圆心
    Pose2d circle_center;
    circle_center.first = (end_k*end_x - start_k*circle_end_pose.first + circle_end_pose.second - end_y)/(end_k - start_k);
    circle_center.second = end_k*(circle_center.first - end_x) + end_y;

    LOG(INFO) <<"circle_center " <<circle_center.first <<" "<<circle_center.second;
    //计算回转半径 若小于最小值 直接返回
    double dis_r = SqrtDistance(circle_center , circle_end_pose);
    if(dis_r < min_corner_r)
    {
        LOG(ERROR)<<"corn r is small , r is " << dis_r;
        return false;
    }
    LOG(INFO) <<"corn r : "<<dis_r;
    /*
        断两个向量之间夹角是顺时针还是逆时针
        利用平面向量的叉乘
        a = (x1,y1) b = (x2,y2)
        a×b = x1y2 - x2y1
        若结果为正，则向量b在a的逆时针方向
        否则，b在a的顺时针方向
        若结果为0，则a与b共线
    */
    //clock_wize = true 顺时针，否则逆时针
    bool clock_wize = false;
    double corn_theta = getAngelOfTwoVector(circle_end_pose , circle_start_pose , circle_center , clock_wize);
    //if(corn_theta > PI) return false;
    corn_theta *= R2D; //转换成角度
    LOG(INFO) <<"corn theta : "<<corn_theta;
    
    //倒车轨迹录点
    parking_path_.clear();
    //起始走一段直线 100等分
    double s_i = begin_line_distance/100.0;
    for(int i=100; i>0 ;i--)
    {
        //在补充直线部分 一百等分
        Pose2d temp_i;
        temp_i.first = circle_start_pose.first + s_i*i*cos(start_ori);
        temp_i.second = circle_start_pose.second + s_i*i*sin(start_ori);
        parking_path_.push_back(temp_i);
    }
    //走圆弧 
    //判断应该是逆时针还是顺时针，向量旋转方程不一样
    if(clock_wize)
    {
        for(int th=(int)2*corn_theta; th>0 ; th--)
        {
            double i = th/2.0;
            Pose2d temp_i;
            //向量法 o->i = o->center + center->i   .  center->i = rot(i*dir)*(end->center)
            temp_i.first = circle_center.first + cos(i*D2R)*(circle_end_pose.first - circle_center.first) 
                                            - sin(-i*D2R)*(circle_end_pose.second - circle_center.second);
            temp_i.second = circle_center.second + sin(-i*D2R)*(circle_end_pose.first - circle_center.first)
                                                + cos(i*D2R)*(circle_end_pose.second - circle_center.second);
            //防止圆弧过调
            //double temp_start = SqrtDistance(temp_i , circle_start_pose);
            //if(temp_start < 0.2) continue;
            parking_path_.push_back(temp_i);
        }
    }
    else
    {
        for(int th=(int)2*corn_theta; th>0 ; th--)
        {
            double i = th/2.0;
            Pose2d temp_i;
            temp_i.first = circle_center.first + cos(i*D2R)*(circle_end_pose.first - circle_center.first) 
                                            - sin(i*D2R)*(circle_end_pose.second - circle_center.second);
            temp_i.second = circle_center.second + sin(i*D2R)*(circle_end_pose.first - circle_center.first)
                                                + cos(i*D2R)*(circle_end_pose.second - circle_center.second);
            
            parking_path_.push_back(temp_i);
        }        
    }

    //走直线
    double s_l = line_distance/100.0;
    for(int i=0; i<100 ;i++)
    {
        Pose2d temp_i;
        temp_i.first = circle_end_pose.first - s_l*i*cos(end_ori);
        temp_i.second = circle_end_pose.second - s_l*i*sin(end_ori);
        parking_path_.push_back(temp_i);
    }

    //检查终点方向与计算得到的方向偏角，不能太大 由于是倒车，所以从 [0]->[2]是近似方向
    double end_direction = RoudingRad(atan2((parking_path_[0].second - parking_path_[2].second),
                                    (parking_path_[0].first - parking_path_[2].first)));
    LOG(INFO) << "end_direction "<<end_direction << "  start dir " << start_ori;
    if(abs(RoudingRad(end_direction - start_ori)) > PI/3.0)
    {
        //计算角度大于阈值 60度 说明起点方向不对
        LOG(ERROR)<< "dir error";
        return false;
    }
    
    
    //出车轨迹就是其倒过来
    outing_path_.clear();
    outing_path_ = parking_path_;
    std::reverse(outing_path_.begin() , outing_path_.end());

    if((parking_path_.size() > 1 )&&(outing_path_.size() > 1))
        return true;
    else
        return false;
    
}

bool CreatPath::GetParkingPath(std::vector<Pose2d>& parkingpath)
{
    if(parking_path_.size() <= 1)
    {
        LOG(WARNING)<<"no parking path, not use this function";
        return false;
    }
    parking_path_.push_back(overshoot_end_pose_);
    parkingpath = parking_path_;
    return true;
}

bool CreatPath::GetOutingPath(std::vector<Pose2d>& outingpath)
{
    if(outing_path_.size()<=1)
    {
        LOG(WARNING)<<"no outing path, not use this function";
        return false;        
    }
    outing_path_.push_back(overshoot_begin_pose_);
    outingpath = outing_path_;
    return true;
}

bool CreatPath::ResetPathData()
{
    parking_path_.clear();
    outing_path_.clear();
}

