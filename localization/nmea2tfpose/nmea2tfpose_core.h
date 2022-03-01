/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef NMEA2TFPOSE_CORE_H
#define NMEA2TFPOSE_CORE_H

// C++ includes
#include <string>
#include <memory>
#include <fstream>
#include <sstream>
#include <queue>
// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Point.h>
#include <nmea_msgs/Sentence.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "geo_pos_conv.h"
#include "../waypoint/wp_core.h"
using namespace std;

namespace gnss_localizer
{
class Nmea2TFPoseNode
{
public:
  Nmea2TFPoseNode();
  ~Nmea2TFPoseNode();

  void run();

private:
  ofstream outfile_;
  ifstream waypointfile_;
  way_point::Gnss_Zmap* wpn;
  bool isGnssReady = false;
  bool useGPSInImu;
  bool pub_imu_enable_ = false;
  bool stopWhileNopose_ = true;
  
  // handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // publisher
  ros::Publisher pub1_;
  ros::Publisher pub2_;
  ros::Publisher pub3_;

  // subscriber
  ros::Subscriber sub1_;
  ros::Subscriber sub2_;
  ros::Subscriber sub3_;
  ros::Subscriber sub4_;

  // constants
  const std::string MAP_FRAME_;
  const std::string GPS_FRAME_;
  const std::string IMU_FRAME_;

  // variables
  double longitude_;
  double latitude_;
  double h_;
  queue<double> qacc_X, qacc_Y, qacc_Z;
  double base_X_, base_Y_, base_Z_;

  double roll_tmp_, pitch_tmp_, yaw_tmp_;
  ros::Time update_last_time_;
  
  geo_pos_conv geo_;
  geo_pos_conv last_geo_;

  double roll_, pitch_, yaw_;
  double gyroX_, gyroY_, gyroZ_;
  double accX_, accY_, accZ_;
  double orientation_time_, position_time_;
  ros::Time current_time_, orientation_stamp_, pose_time_;
  std::string pose_topic_;
  std::string waypoint_path_;
  std::string pointTopic_;
  tf::TransformBroadcaster br_;

  tf::TransformListener transform_listener;
  tf::StampedTransform tf;
  // callbacks
  void callbackFromNmeaSentence(const nmea_msgs::Sentence::ConstPtr &msg);
  void callbackSetOrigin(const std_msgs::Int8::ConstPtr& flag);
  void callbackPose(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void callbackGps(const sensor_msgs::NavSatFix::ConstPtr &msg);
  
  // initializer
  void initForROS();

  // functions
  void publishPoseStamped();
  void publishPointStamped();
  void publishImuData();
  void publishTF();
  void createOrientation();
  int convert(std::vector<std::string> nmea, ros::Time current_stamp);
};


std::vector<std::string> split(const std::string &string);

}  // gnss_localizer
#endif  // NMEA2TFPOSE_CORE_H
