#pragma once
#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <opencv/cv.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

using namespace std;

typedef pcl::PointXYZI PointType;

enum class SensorType { VELODYNE, OUSTER };

class ParamServer
{
public:

    ros::NodeHandle nh;


    //Topics
    string imuTopic;
    string odomTopic;

    //Frames
    string odometryFrame;
    string mapFrame;

    // GPS Settings
    float gpsCovThreshold;
    float poseCovThreshold;

    // Save pcd
    bool savePCD;
    string savePCDDirectory;

    // Lidar Sensor Configuration
    SensorType sensor;

    // IMU
    float imuAccNoise;
    float imuGyrNoise;
    float imuAccBiasN;
    float imuGyrBiasN;
    float imuGravity;
    float imuRPYWeight;
    float initialPubGate;
    vector<double> extRotV;
    vector<double> extRPYV;
    vector<double> extTransV;
    Eigen::Matrix3d extRot;
    Eigen::Matrix3d extRPY;
    Eigen::Vector3d extTrans;
    Eigen::Quaterniond extQRPY;

    // LOAM
    float edgeThreshold;
    float surfThreshold;
    int edgeFeatureMinValidNum;
    int surfFeatureMinValidNum;

    // voxel filter paprams
    float odometrySurfLeafSize;
    float mappingCornerLeafSize;
    float mappingSurfLeafSize ;

    float z_tollerance; 
    float rotation_tollerance;

    // CPU Params
    int numberOfCores;
    double mappingProcessInterval;

    // Surrounding map
    float surroundingkeyframeAddingDistThreshold; 
    float surroundingkeyframeAddingAngleThreshold; 
    float surroundingKeyframeDensity;
    float surroundingKeyframeSearchRadius;
    
    // Loop closure
    bool  loopClosureEnableFlag;
    float loopClosureFrequency;
    int   surroundingKeyframeSize;
    float historyKeyframeSearchRadius;
    float historyKeyframeSearchTimeDiff;
    int   historyKeyframeSearchNum;
    float historyKeyframeFitnessScore;
    float keyFrameDistFitnessScore;
    float keyFrameExtFitnessScore;
    float updateFrequency;
    

    // global map visualization radius
    float globalMapVisualizationSearchRadius;
    float globalMapVisualizationPoseDensity;
    float globalMapVisualizationLeafSize;

    ParamServer()
    {

        nh.param<std::string>("imuTopic", imuTopic, "imu/data");
        nh.param<std::string>("odomTopic", odomTopic, "odometry/imu");
        nh.param<std::string>("odometryFrame", odometryFrame, "odom");
        nh.param<std::string>("mapFrame", mapFrame, "map");

        nh.param<float>("gpsCovThreshold", gpsCovThreshold, 2.0);
        nh.param<float>("poseCovThreshold", poseCovThreshold, 25.0);
        nh.param<float>("imuAccNoise", imuAccNoise, 0.01); 
        nh.param<float>("imuGyrNoise", imuGyrNoise, 0.001);
        nh.param<float>("imuAccBiasN", imuAccBiasN, 0.0002);
        nh.param<float>("imuGyrBiasN", imuGyrBiasN, 0.00003);
        nh.param<float>("imuGravity", imuGravity, 9.80511);
        nh.param<float>("imuRPYWeight", imuRPYWeight, 0.01);

        nh.param<float>("initialPubGate", initialPubGate, 0.3); 
        
        nh.param<vector<double>>("extrinsicRot", extRotV, vector<double>());
        nh.param<vector<double>>("extrinsicRPY", extRPYV, vector<double>());
        nh.param<vector<double>>("extrinsicTrans", extTransV, vector<double>());

        extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
        extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
        extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
        extQRPY = Eigen::Quaterniond(extRPY);

        nh.param<int>("lio_sam/numberOfCores", numberOfCores, 2);

        nh.param<float>("updateFrequency", updateFrequency, 50.0);


        usleep(100);
    }

    sensor_msgs::Imu imuConverter(const sensor_msgs::Imu& imu_in)
    {
        sensor_msgs::Imu imu_out = imu_in;
        // rotate acceleration
        Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
        acc = extRot * acc;
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();
        // rotate gyroscope
        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
        gyr = extRot * gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();
        // rotate roll pitch yaw
        Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
        Eigen::Quaterniond q_final = q_from * extQRPY;
        imu_out.orientation.x = q_final.x();
        imu_out.orientation.y = q_final.y();
        imu_out.orientation.z = q_final.z();
        imu_out.orientation.w = q_final.w();

        if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
        {
            ROS_ERROR("Invalid quaternion, please use a 9-axis IMU!");
            ros::shutdown();
        }

        return imu_out;
    }


    geometry_msgs::PoseStamped poseConverter(const geometry_msgs::PoseStamped& pose_in)
    {
        geometry_msgs::PoseStamped pose_out = pose_in;
        // rotate pose
        Eigen::Vector3d position(pose_in.pose.position.x, pose_in.pose.position.y, pose_in.pose.position.z);
        pose_out.pose.position.x = position.x();
        pose_out.pose.position.y = position.y();
        pose_out.pose.position.z = position.z();
        // rotate roll pitch yaw
        // Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
        // Eigen::Quaterniond q_final = q_from * extQRPY;
        // imu_out.orientation.x = q_final.x();
        // imu_out.orientation.y = q_final.y();
        // imu_out.orientation.z = q_final.z();
        // imu_out.orientation.w = q_final.w();


        return pose_out;
    }

    geometry_msgs::PoseStamped pointConverter(const geometry_msgs::PointStamped& point_in)
    {
        geometry_msgs::PoseStamped pose_out;
        pose_out.header = point_in.header;
        // rotate pose
        Eigen::Vector3d position(point_in.point.x, point_in.point.y, point_in.point.z);
        pose_out.pose.position.x = position.x();
        pose_out.pose.position.y = position.y();
        pose_out.pose.position.z = position.z();
        // rotate roll pitch yaw
        pose_out.pose.orientation.x = 0;
        pose_out.pose.orientation.y = 0;
        pose_out.pose.orientation.z = 0;
        pose_out.pose.orientation.w = 1;


        return pose_out;
    }
};


sensor_msgs::PointCloud2 publishCloud(ros::Publisher *thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub->getNumSubscribers() != 0)
        thisPub->publish(tempCloud);
    return tempCloud;
}

template<typename T>
double ROS_TIME(T msg)
{
    return msg->header.stamp.toSec();
}


template<typename T>
void imuAngular2rosAngular(sensor_msgs::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z)
{
    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}


template<typename T>
void imuAccel2rosAccel(sensor_msgs::Imu *thisImuMsg, T *acc_x, T *acc_y, T *acc_z)
{
    *acc_x = thisImuMsg->linear_acceleration.x;
    *acc_y = thisImuMsg->linear_acceleration.y;
    *acc_z = thisImuMsg->linear_acceleration.z;
}


template<typename T>
void imuRPY2rosRPY(sensor_msgs::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw)
{
    double imuRoll, imuPitch, imuYaw;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(thisImuMsg->orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

    *rosRoll = imuRoll;
    *rosPitch = imuPitch;
    *rosYaw = imuYaw;
}


float pointDistance(PointType p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}


float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

float pointDistance(geometry_msgs::PoseStamped p1, geometry_msgs::PoseStamped p2)
{
    return sqrt((p1.pose.position.x-p2.pose.position.x)*(p1.pose.position.x-p2.pose.position.x) + (p1.pose.position.y-p2.pose.position.y)*(p1.pose.position.y-p2.pose.position.y) + (p1.pose.position.z-p2.pose.position.z)*(p1.pose.position.z-p2.pose.position.z));
}

//获取两个位置的距离
double GetDifferenceTwoPose(geometry_msgs::Pose poseA, geometry_msgs::Pose poseB) {
    tf::Quaternion orientationA, orientationB;
    tf::quaternionMsgToTF(poseA.orientation, orientationA);
    tf::quaternionMsgToTF(poseB.orientation, orientationB);
    double rot = 2*acos(abs(orientationA.dot(orientationB)));
    cout << "rot:" << rot << endl;
    return sqrt((poseA.position.y - poseB.position.y) * (poseA.position.y - poseB.position.y)
        + (poseA.position.x - poseB.position.x) * (poseA.position.x - poseB.position.x)) + 10* rot * rot;
}



#endif
