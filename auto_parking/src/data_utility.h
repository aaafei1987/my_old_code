/*
 * @Author: your name
 * @Date: 2021-09-30 08:59:30
 * @LastEditTime: 2021-11-04 15:05:51
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /auto_parking/src/data_utility.h
 */

#ifndef DATA_UTILITY_H_
#define DATA_UTILITY_H_

//主要用来存头文件 公共的
#include <iostream>
#include <ros/ros.h>
#include <ctime>
#include <cassert>
#include <cmath>
#include <utility>
#include <vector>
#include <algorithm> 
#include <cstdlib>
#include <memory>
#include <thread>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include "glog/logging.h"

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <nav_msgs/Odometry.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>


#include <auto_parking/AutoParkStatus.h>
#include <auto_parking/SetAutoPark.h>

const double PI = 3.1415926535897932384; ///< PI
const double D2R = 0.017453292519943; 
const double R2D = 57.29577951308232; 

#define Pose2d std::pair<double,double> 
//第一个值是线速度 第二个值是角速度 数据格式与pose一样
#define Velocity2d std::pair<double,double>

enum ParkStatus{
    NO_MOVE = 0 ,  //状态是未运动
    FORWARD_MOVE = 1 , //向前运动中，一般是退出倒车时
    FORWARD_PAUSE = 2 , //向前运动中触发暂停
    BACKWARD_MOVE = 3 , //向后运动中 ， 倒车中
    BACKWARD_PAUSE = 4,   //倒车中暂停
    STOP = 5 //出故障暂停
};

inline double SqrtDistance(Pose2d x1 , Pose2d x2)
{
    return sqrt(pow(x1.first-x2.first , 2)+pow(x1.second - x2.second , 2));
}

//rad 元整到 （-pi，pi]之间
inline double RoudingRad(double input_rad)
{
    double output_rad = input_rad;
    while(output_rad > PI)
        output_rad -= 2*PI;
    while(output_rad <= -PI)
        output_rad += 2*PI;
    return output_rad;
}

//向量夹角 [0, 2pi] 
inline double getAngelOfTwoVector(Pose2d p1, Pose2d p2 ,Pose2d c , bool &clock_wise)
{
    Pose2d pt1;
    pt1.first = p1.first - c.first;
    pt1.second = p1.second - c.second;
    Pose2d pt2;
    pt2.first = p2.first - c.first;
    pt2.second = p2.second - c.second;
    // a×b = x1y2 - x2y1
    double axb = pt1.first*pt2.second - pt2.first*pt1.second;
    if(axb <= 0) 
        clock_wise = true;
    else 
        clock_wise = false;

    double delta = atan2(pt1.second , pt1.first) - atan2(pt2.second , pt2.first);
    if(delta <0)
        delta +=2*PI;
    if(delta > PI) 
        delta = 2*PI - delta;
    return delta;  
}

#endif