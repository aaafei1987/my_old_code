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

#include "nmea2tfpose_core.h"
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>

const double D2R = 0.017453292519943;
const int SEPERATE_NUM = 8;

namespace gnss_localizer
{
  long getTimestamp()
  {
    struct timeval tv;
 
    gettimeofday(&tv, NULL);
     
    return (tv.tv_sec*1000 + tv.tv_usec/1000);
  }

string makeFileName()
{
	char buff[128] = {0};
	time_t tt;
	struct tm tr = {0};
	time(&tt);
	localtime_r(&tt, &tr);
	snprintf(buff, sizeof(buff),
		"/home/a/log/GPS_DAT_%02d%02d%"
		"02d%02d%02d.csv", 
		tr.tm_mon + 1, tr.tm_mday, 
		tr.tm_hour, tr.tm_min, tr.tm_sec);
	
	string filename = buff;
	return filename;
}

// Constructor
Nmea2TFPoseNode::Nmea2TFPoseNode()
  : private_nh_("~")
  , MAP_FRAME_("/map")
  , GPS_FRAME_("/gnss")
  , IMU_FRAME_("/imu")
  , roll_(0)
  , pitch_(0)
  , yaw_(0)
  , yaw_tmp_(0)
  , gyroX_(0)
  , gyroY_(0)
  , gyroZ_(0)
  , accX_(0)
  , accY_(0)
  , accZ_(0)
  , base_X_(0.0)
  , base_Y_(0.0)
  , base_Z_(1.0)
  , orientation_time_(0)
  , position_time_(0)
  , current_time_(0)
  , update_last_time_(0)
  , orientation_stamp_(0)
{
  isGnssReady = false;
  useGPSInImu = false;
  pub_imu_enable_ = false;
  stopWhileNopose_ = true;
  wpn = new way_point::Gnss_Zmap;
  initForROS();
  transform_listener.waitForTransform(MAP_FRAME_, GPS_FRAME_, ros::Time(0), ros::Duration(1.0));
  transform_listener.lookupTransform(MAP_FRAME_, GPS_FRAME_, ros::Time(0), tf);
}

// Destructor
Nmea2TFPoseNode::~Nmea2TFPoseNode()
{
  outfile_.close();
}

void Nmea2TFPoseNode::initForROS()
{
  // ros parameter setting
  //从参数服务器初始化话题
  //log
  #if 1
  longitude_ = 117.9776288;
  latitude_ = 24.599862;
  waypoint_path_ = "/home/a/install/lib/connector/.data/map.csv";
  private_nh_.getParam("longitude", longitude_);
  private_nh_.getParam("latitude", latitude_);
  private_nh_.getParam("useGPSInImu",useGPSInImu);
  private_nh_.getParam("stopWhileNopose",stopWhileNopose_);
  private_nh_.getParam("pose_topic",pose_topic_);
  private_nh_.getParam("point_topic",pointTopic_);
  private_nh_.getParam("waypoint_path",waypoint_path_);
  ROS_INFO("lon:%lf, lat:%lf", longitude_, latitude_); 
  geo_.set_origin(longitude_, latitude_);
  #endif
  // setup subscriber
  sub1_ = nh_.subscribe("nmea_sentence", 100, &Nmea2TFPoseNode::callbackFromNmeaSentence, this);
  sub2_ = nh_.subscribe("set_start_point", 10, &Nmea2TFPoseNode::callbackSetOrigin,this);
  sub3_ = nh_.subscribe(pose_topic_, 10, &Nmea2TFPoseNode::callbackPose,this);
  sub3_ = nh_.subscribe("/gx5/gps/fix", 10, &Nmea2TFPoseNode::callbackGps,this);

  // setup publisher
  pub1_ = nh_.advertise<geometry_msgs::PoseStamped>("gnss_pose", 1000);
  pub2_ = nh_.advertise<sensor_msgs::Imu>("imu_raw1", 1000);
  pub3_ = nh_.advertise<geometry_msgs::PointStamped>(pointTopic_, 1000);

  string filename = makeFileName();
  ROS_INFO_STREAM("GPS filename:" << filename); 
  outfile_.open(filename.c_str(), ios::out);
  wpn->get_vector_from_file(waypoint_path_);
}

void Nmea2TFPoseNode::run()
{
  ros::spin();
}

void Nmea2TFPoseNode::publishPoseStamped()
{
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = MAP_FRAME_;
  pose.header.stamp = current_time_;
  pose.pose.position.x = geo_.x();
  pose.pose.position.y = geo_.y();

  pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll_, pitch_, yaw_);
  if(useGPSInImu)
    yaw_tmp_ = 0;

#if 1
  tf::Pose tf_pose;
  tf::poseMsgToTF(pose.pose, tf_pose);
  tf_pose = tf * tf_pose;
  tf::poseTFToMsg(tf_pose,pose.pose);
#endif  
  // double x_tmp = 0.99994 * pose.pose.position.x - 0.0109399 * pose.pose.position.y + 0.000109848 *pose.pose.position.z + 0.91633;
  // double y_tmp = 0.01094 * pose.pose.position.x + 0.9994 * pose.pose.position.y - 0.000853445 *pose.pose.position.z + 0.44608;
  // double z_tmp = -0.0001005 * pose.pose.position.x + 0.0008546 * pose.pose.position.y - 0.999999 *pose.pose.position.z - 0.46072;

  // pose.pose.position.x = x_tmp;
  // pose.pose.position.y = y_tmp;
  // pose.pose.position.z = z_tmp;
  geometry_msgs::Point point;
  point = pose.pose.position;
  point = wpn->gnss_remapping(point);
  pose.pose.position.z = point.z;

  pub1_.publish(pose);
 
  ROS_INFO("Pose: x:%lf, y:%lf, yaw:%lf.", point.x, point.y, yaw_); 
  outfile_ << geo_.x() << "," 
           << geo_.y() << ","
           << geo_.z() << ","
           << roll_ << ","
           << pitch_ << ","
           << yaw_ << endl;
}


void Nmea2TFPoseNode::publishPointStamped()
{
  geometry_msgs::PoseStamped pose;
  geometry_msgs::PointStamped point;
  point.header.frame_id = MAP_FRAME_;
  point.header.stamp = current_time_;
  pose.pose.position.x = geo_.x();
  pose.pose.position.y = geo_.y();

  if(useGPSInImu)
    yaw_tmp_ = 0;

#if 1
  tf::Pose tf_pose;
  tf::poseMsgToTF(pose.pose, tf_pose);
  tf_pose = tf * tf_pose;
  tf::poseTFToMsg(tf_pose,pose.pose);
#endif  
  // double x_tmp = 0.99994 * pose.pose.position.x - 0.0109399 * pose.pose.position.y + 0.000109848 *pose.pose.position.z + 0.91633;
  // double y_tmp = 0.01094 * pose.pose.position.x + 0.9994 * pose.pose.position.y - 0.000853445 *pose.pose.position.z + 0.44608;
  // double z_tmp = -0.0001005 * pose.pose.position.x + 0.0008546 * pose.pose.position.y - 0.999999 *pose.pose.position.z - 0.46072;

  // pose.pose.position.x = x_tmp;
  // pose.pose.position.y = y_tmp;
  // pose.pose.position.z = z_tmp;
  geometry_msgs::Point pnt;
  pnt = pose.pose.position;
  pnt = wpn->gnss_remapping(pnt);
  point.point= pnt;

  pub3_.publish(point);
 
}

void Nmea2TFPoseNode::publishImuData()
{
  sensor_msgs::Imu imu;
  ros::Duration d_tmp = (current_time_ - update_last_time_);
  ros::Duration pose_missing = (current_time_ - pose_time_);
  imu.header.frame_id = IMU_FRAME_;
  imu.header.stamp = current_time_;

  imu.angular_velocity.x = gyroY_*D2R;
  imu.angular_velocity.y = -gyroX_*D2R;
  imu.angular_velocity.z = gyroZ_*D2R;
  
  if(pub_imu_enable_ && qacc_X.size()> 50)
  {
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;
    for(int i=0; i<qacc_X.size(); ++i)
    {
      sum_x +=qacc_X.front();
      qacc_X.pop();
      sum_y +=qacc_Y.front();
      qacc_Y.pop();
      sum_z +=qacc_Z.front();
      qacc_Z.pop();
    }
    base_X_ = sum_x/qacc_X.size();
    base_Y_ = sum_y/qacc_Y.size();
    base_Z_ = sum_z/qacc_Z.size();
  }
  else if(pub_imu_enable_);//开始发布imu信息后不再进行入队出队
  else if(qacc_X.size() > 100)
  {
    qacc_X.push(accX_);
    qacc_Y.push(accY_);
    qacc_Z.push(accZ_);
    qacc_X.pop();
    qacc_Y.pop();
    qacc_Z.pop();
  }
  else
  {
    qacc_X.push(accX_);
    qacc_Y.push(accY_);
    qacc_Z.push(accZ_);
  }
  

  imu.linear_acceleration.x = (accX_ - base_X_)*9.8;
  imu.linear_acceleration.y = (accY_ - base_Y_)*9.8;
  imu.linear_acceleration.z = (1+accZ_-base_Z_)*9.8;
  if(useGPSInImu)
  {
    yaw_tmp_ += imu.angular_velocity.z * d_tmp.toSec();
    imu.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll_, pitch_, yaw_ + yaw_tmp_);
  }
  else
  {
    roll_tmp_ += imu.angular_velocity.x * d_tmp.toSec();
    pitch_tmp_ += imu.angular_velocity.y * d_tmp.toSec();
    yaw_tmp_ += imu.angular_velocity.z * d_tmp.toSec();
    imu.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll_tmp_, pitch_tmp_, yaw_tmp_);
  }

  if (pub_imu_enable_ == false)
  {
    roll_tmp_ = 0;
    pitch_tmp_ = 0;
    yaw_tmp_ = 0;
    return;
  }
  else
  {
    pub2_.publish(imu);
    if(stopWhileNopose_ && pose_missing.toSec() > 10)
    {
      pub_imu_enable_ = false;
    }
  }
  //ROS_INFO("IMU: wx:%lf, wy:%lf, wz:%lf, ax:%lf, ay:%lf, az:%lf.", 
  //    gyroX_, gyroY_, gyroZ_, accX_, accY_, accZ_); 
}

void Nmea2TFPoseNode::publishTF()
{
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(geo_.x(), geo_.y(), geo_.z()));
  tf::Quaternion quaternion;
  quaternion.setRPY(roll_, pitch_, yaw_);
  transform.setRotation(quaternion);
  br_.sendTransform(tf::StampedTransform(transform, current_time_, MAP_FRAME_, GPS_FRAME_));
}

void Nmea2TFPoseNode::createOrientation()
{
  yaw_ = atan2(geo_.y(), geo_.x());
  roll_ = 0;
  pitch_ = 0;
}

int Nmea2TFPoseNode::convert(std::vector<std::string> nmea, ros::Time current_stamp)
{
  int count = nmea.size();
  try
  {
    if (nmea.at(0) == "$GPFPD")  //heading pitch roll
    {
      if ((nmea.at(15).substr(0,2) != "4B") && (nmea.at(15).substr(0,2) != "0B"))
      {
        ROS_WARN_STREAM("NewTon Status is not RTK now.");
        return -1;
      }
      if (isGnssReady == false)
      {
	  isGnssReady = true;
      }
	   

      position_time_ = stod(nmea.at(2));
      roll_ = stod(nmea.at(5)) * M_PI / 180.;
      pitch_ = -1 * stod(nmea.at(4)) * M_PI / 180.;
      yaw_ = -stod(nmea.at(3)) * M_PI / 180. + M_PI*2;
      
	//    ROS_INFO("roll:%f, pitch:%f, yaw:%f.", roll_,pitch_,yaw_);

      double lat = stod(nmea.at(6));
      double lon = stod(nmea.at(7));
      double h = stod(nmea.at(8));
 
      if ( geo_.hasInitialized())
      {
      	geo_.set_llh_nmea_degrees(lat, lon, h);
	return 2;
      }
      else
      {
        longitude_ = lon;
        latitude_ = lat;
        h_ = h;
      }
    }
    else if (nmea.at(0) == "$GPHPD")
    {
      position_time_ = stod(nmea.at(2));
      roll_ = stod(nmea.at(5)) * M_PI / 180.;
      pitch_ = -1 * stod(nmea.at(4)) * M_PI / 180.;
      yaw_ = -stod(nmea.at(3)) * M_PI / 180. + M_PI*2;

      double lat = stod(nmea.at(6));
      double lon = stod(nmea.at(7));
      double h = stod(nmea.at(8));
      geo_.set_llh_nmea_degrees(lat, lon, h);
    }
    else if (nmea.at(0) == "$GTIMU")
    {
      gyroX_ = stod(nmea.at(3));
      gyroY_ = stod(nmea.at(4));
      gyroZ_ = stod(nmea.at(5));

      accX_ = stod(nmea.at(6));
      accY_ = stod(nmea.at(7));
      accZ_ = stod(nmea.at(8));

      return 1;
    }
  }catch (const std::exception &e)
  {
    ROS_WARN_STREAM("Message is invalid : " << e.what());
    return -1;
  }
  return 0;
}

void Nmea2TFPoseNode::callbackFromNmeaSentence(const nmea_msgs::Sentence::ConstPtr &msg)
{
  int res = 0;
  res = convert(split(msg->sentence), msg->header.stamp) ;
  if ( res < 0)
  {
      ROS_WARN_STREAM("Msg Err : " << msg->sentence);
      return ;
  }


  if (res == 1)
  {
    if(current_time_ == ros::Time(0) || (!isGnssReady && useGPSInImu))
      update_last_time_ = msg->header.stamp;
    else
      update_last_time_ = current_time_;
    current_time_ = msg->header.stamp;
    publishImuData();
  }
  else if (res == 2)
  {
    if(!isGnssReady && useGPSInImu)
      update_last_time_ = msg->header.stamp;
    else if (useGPSInImu)
      update_last_time_ = current_time_;
    current_time_ = msg->header.stamp;
    publishPoseStamped();
    //publishTF();
    //last_geo_ = geo_;
  }
  
  return;
}

void Nmea2TFPoseNode::callbackPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  if(stopWhileNopose_)
    pose_time_ = msg->header.stamp;
  else
    pose_time_ = ros::Time::now();
  pub_imu_enable_ = true;
  return;
}

void Nmea2TFPoseNode::callbackGps(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
  
  double lat = msg -> latitude;
  double lon = msg -> longitude;
  double h = msg -> altitude;
  geo_.set_llh_nmea_degrees(lat, lon, h);
  publishPointStamped();
  return;
}

void Nmea2TFPoseNode::callbackSetOrigin(const std_msgs::Int8::ConstPtr& msg)
{
	ROS_INFO("recv msg to set start point.....");
	if (isGnssReady == false)
		return;

	if ( msg->data == 0)
		return;
	
	geo_.set_origin(latitude_, longitude_);
}

std::vector<std::string> split(const std::string &string)
{
  std::vector<std::string> str_vec_ptr;
  std::string token;
  std::stringstream ss(string);

  while (getline(ss, token, ','))
    str_vec_ptr.push_back(token);

  return str_vec_ptr;
}

}  // gnss_localizer

/*
本段用于从map.csv文件中获取距离当前位置最近的点，并以此点位z坐标替换当前gnss中的z坐标，减少地图扭曲造成的定位误差
*/

namespace way_point
{

std::vector<std::string> split(const std::string &string)
{
  std::vector<std::string> str_vec_ptr;
  std::string token;
  std::stringstream ss(string);

  while (getline(ss, token, ','))
    str_vec_ptr.push_back(token);

  return str_vec_ptr;
}


Gnss_Zmap::Gnss_Zmap()
  : root(nullptr)
{
    point_list = std::vector<geometry_msgs::Point>();
}

Gnss_Zmap::~Gnss_Zmap()
{
    point_list.clear ();
}
void Gnss_Zmap::run()
{
  ros::spin();
}

geometry_msgs::Point Gnss_Zmap::string2point(std::string point_line)
{
  std::vector<std::string> string_ptr = split(point_line);
  geometry_msgs::Point pnt;
  pnt.x = stod(string_ptr.at(0));
  pnt.y = stod(string_ptr.at(1));
  pnt.z = stod(string_ptr.at(2));
  return pnt;
}

std::vector<geometry_msgs::Point> Gnss_Zmap::get_vector_from_file(std::string filename)
{
  ifstream infile(filename.c_str());
  if(!infile){
      cout<<"open file fail!"<<endl;
      return std::vector<geometry_msgs::Point>();
  }
  string str;
  getline(infile,str);          //去掉第一行，表头
  while( getline(infile,str))
  {
    geometry_msgs::Point point = Gnss_Zmap::string2point(str);
    point_list.push_back(point);
    ROS_INFO("point.x=%f,point.y=%f,point.z=%f",point.x,point.y,point.z);
  }
  
  infile.close();
  return point_list;
}


geometry_msgs::Point Gnss_Zmap::gnss_remapping(geometry_msgs::Point pose)
{
  if(point_list.size() == 0)
  {
    //cout<<"no tree to search,remapping fail!"<<endl;
    return pose;
  }
  geometry_msgs::Point node = Gnss_Zmap::search(point_list,pose);
  pose.z = node.z;
  return pose;
}

geometry_msgs::Point Gnss_Zmap::search(std::vector<geometry_msgs::Point> point_list, geometry_msgs::Point point)
{
	if (point_list.size() == 0)
	{
    cout<<"Search failed:point_list not initialed!"<<endl;
		return point;
	}

  int vectorsize = point_list.size(); 
  if (vectorsize <= SEPERATE_NUM + 1)
  {
    return point_list[0];
  }
  
  double dist[SEPERATE_NUM+1];
  int min_count = 0;
  for (int i = 0; i < SEPERATE_NUM + 1; i++)
  {
    dist[i] = pow(point.x - point_list[i*(vectorsize-1)/SEPERATE_NUM].x, 2) + pow(point.y - point_list[i*(vectorsize-1)/SEPERATE_NUM].y, 2);
  }
  double minsum= dist[0] + dist[1];
  for (int i = 0; i < SEPERATE_NUM; ++i)
  {
    if(dist[i] + dist[i+1] < minsum)
    {
      minsum = dist[i] + dist[i+1];
      min_count = i;
    }
  }
  return Gnss_Zmap::search(std::vector<geometry_msgs::Point>(point_list.begin() + min_count * vectorsize/SEPERATE_NUM, point_list.begin() + (min_count+1) * vectorsize / SEPERATE_NUM),point);

}
}



