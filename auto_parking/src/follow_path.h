/*
 * @Author: your name
 * @Date: 2021-09-30 09:03:52
 * @LastEditTime: 2022-01-12 16:59:57
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /auto_parking/src/follow_path.h
 */

#ifndef FOLLOW_PATH_H_
#define FOLLOW_PATH_H_

#include "data_utility.h"


class FollowPath{

    //该类 需要： 1当前位置  2轨迹  3前进还是后退指令 
    //调用顺序 1 initconfig 2set waypoints 3setcurpose 4setStartEnd  5setCarOri  6getvel 结束后 7resetState
    //每一次新的巡线 1setwaypoints
    public:
    
        FollowPath();
        bool InitConfig(const YAML::Node yaml_node);
        //获取线速度与角速度 若能算出结果则返回true
        bool GetVelocity(Velocity2d &cmd_vel);
        //中断跟踪
        bool PauseParking();
        //继续跟踪
        bool ContinueParking();
        //重置 以便再次倒车
        void ResetState();
        //检测是否到达终点
        bool CheckFinished();

        //首先调用这个函数，获取所有轨迹 （可以包含以及走过的）默认从头走到尾
        bool SetWaypoints(std::vector<Pose2d> waypoints)
        {
            waypoints_ = waypoints;
            if(waypoints_.size() < 2) 
                return false;
            have_waypoints_ = true;
            return true;
        }
        //获取当前的坐标 需要一直调用
        void SetCurrentPose(Pose2d current_pose)
        {
            cur_time = ros::Time::now();
            current_pose_ = current_pose;
        /* 计算速度
            if(cur_time.toSec() > last_time.toSec())
            {
                static double acc_dis = 0.0;
                static double acc_delta_time = 0.0;
                static int acc_nums = 0;
                double dis = sqrt(pow(last_pose_.first - current_pose_.first , 2) + 
                                        pow(last_pose_.second - current_pose_.second , 2));
                acc_nums ++;
                acc_dis += dis;
                acc_delta_time += (cur_time.toSec() - last_time.toSec());
                if(acc_nums > 3)
                {
                    //三次计算一次速度
                    acc_nums = 0;
                    current_vel_ = acc_dis/acc_delta_time;
                    acc_delta_time = 0.0;
                    acc_dis = 0.0;
                }         
            }
            last_pose_ = current_pose_;
            last_time = cur_time; 
        */
            have_current_pose_ = true;
        }

        //获取当前车辆朝向 （和上面配合用 前进还是后退）弧度制
        void SetCarOrientation(double current_ori)
        {
            //角度范围为 -pi～pi
            current_ori_ = current_ori;
            if(current_ori_ > PI)
                current_ori_ -= PI;
            if(current_ori_ < -PI)
                current_ori_ += PI;
            have_current_ori_ = true;
        }


    private:

        bool CheckSecurity();
        //获取下个目标点（在已有waypoints上）全局存储该nextpose的index
        bool GetNextWaypoints();
        //根据下个点与当前点 计算回转半径(返回值)
        double computeCurvature(Pose2d next_pose);
        //根据回转半径计算angular.z  线速度恒定(暂时这样) bool：前进还是后退
        Velocity2d computeVelocity(double kappa , bool forward);
        //计算第一个点 在第二个点坐标系下的位置
        geometry_msgs::Point calcRelativeCoordinate(Pose2d next_pose, Pose2d current_pose , double current_ori);
        //线速度是恒定的 角速度根据回转半径不同也会改变 v=rw （w是弧度/s)
        double line_vel_; //实际使用的 要到终点会降速
        double line_vel_yaml_; //yaml导入 常规倒车速度
        double lookahead_min_distance_;
        double lookahead_max_distance_;
        double limit_min_angular_vel_;
        double limit_max_angular_vel_;
        double min_trunning_radius_;
        double max_trunning_radius_;
        double tolarte_ori_; //当前方向与轨迹方向偏差的容忍度 
        double start_dis_thread_;
        double distance_to_goal_;
        double distance_slow_to_goal_;
        double vel_to_goal_;
        
        bool forward_; //forward = true 正走 false 倒着走
        bool have_config_;
        bool have_current_pose_;
        bool have_current_ori_;
        bool have_waypoints_;
        //判断是否是在暂停中 如果是的话就保存状态 否的话就清空状态，等待下个路径规划
        bool is_pause_;
        //新进来的轨迹点 要判断从哪块开始走
        bool new_path_flag_;

        std::vector<Pose2d> waypoints_;
        Pose2d current_pose_;
        Pose2d last_pose_;
        double current_ori_;
        double current_vel_; //当前速度，根据pose变化计算出来的
        ros::Time cur_time;
        ros::Time last_time;
        //下一个pose的index
        int next_pose_index_;
        int current_index_;
};


#endif