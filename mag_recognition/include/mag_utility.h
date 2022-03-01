/*
 * @Author: your name
 * @Date: 2021-09-30 08:59:30
 * @LastEditTime: 2022-01-10 10:20:18
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /auto_parking/src/data_utility.h
 */

#ifndef MAG_UTILITY_H_
#define MAG_UTILITY_H_

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
#include "global_defination.h"

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>



const double PI = 3.1415926535897932384; ///< PI
const double D2R = 0.017453292519943; 
const double R2D = 57.29577951308232; 

#define Pose2d std::pair<double,double> 
#define RosPoseType geometry_msgs::PoseWithCovarianceStamped 

inline double SqrtDistance(Pose2d x1 , Pose2d x2)
{
    return sqrt(pow(x1.first-x2.first , 2)+pow(x1.second - x2.second , 2));
}

inline double SqrtDistance(double x1 ,double y1 , double x2 , double y2)
{
    return sqrt(pow(x1-x2, 2)+pow(y1-y2 , 2));
}

struct MagInfoStruct
{
    int mag_index;
    //此时的磁钉信息位置
    struct {
        double pose_x;
        double pose_y;
        double pose_z;
        double orientation_w;
        double orientation_x;
        double orientation_y;
        double orientation_z;
    }mag_pose;
    //此时的磁钉数据
    struct {
        unsigned int node_id;
        int distance_left;
        int distance_middle;
        int distance_right;
        double expend_unused; //留下待使用 为以后扩展
    }mag_data;
};

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


        //点云无穷点去除
        template <typename PointType> void
        removeNaN (const pcl::PointCloud<PointType> &cloud_in, 
                                    pcl::PointCloud<PointType> &cloud_out,
                                    std::vector<int> &index)
        {
            // If the clouds are not the same, prepare the output
            if (&cloud_in != &cloud_out)
            {
                cloud_out.header = cloud_in.header;
                cloud_out.points.resize (cloud_in.points.size ());
            }
            // Reserve enough space for the indices
            index.resize (cloud_in.points.size ());
            size_t j = 0;

            // If the data is dense, we don't need to check for NaN
            if (cloud_in.is_dense)
            {
                // Simply copy the data
                cloud_out = cloud_in;
                for (j = 0; j < cloud_out.points.size (); ++j)
                index[j] = static_cast<int>(j);
            }
            else
            {
                for (size_t i = 0; i < cloud_in.points.size (); ++i)
                {
                if (!pcl_isfinite (cloud_in.points[i].x) || 
                    !pcl_isfinite (cloud_in.points[i].y) || 
                    !pcl_isfinite (cloud_in.points[i].z))
                    continue;
                cloud_out.points[j] = cloud_in.points[i];
                index[j] = static_cast<int>(i);
                j++;
                }
                if (j != cloud_in.points.size ())
                {
                // Resize to the correct size
                cloud_out.points.resize (j);
                index.resize (j);
                }

                cloud_out.height = 1;
                cloud_out.width  = static_cast<uint32_t>(j);

                // Removing bad points => dense (note: 'dense' doesn't mean 'organized')
                cloud_out.is_dense = true;
            }
        }


#endif