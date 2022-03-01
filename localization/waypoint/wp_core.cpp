#include "wp_core.h"
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
/*
本程序包用于从map.csv文件中四叉树检索距离当前位置最近的点
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
    root = nullptr;
}

Gnss_Zmap::~Gnss_Zmap()
{
    clear(root);
}
void Gnss_Zmap::run()
{
  ros::spin();
}

geometry_msgs::Point Gnss_Zmap::string2point(std::string point_line)
{
  std::vector<std::string> string_ptr = split(point_line);
  geometry_msgs::Point pnt;
  pnt.x = stod(string_ptr.at(1));
  pnt.y = stod(string_ptr.at(2));
  pnt.z = stod(string_ptr.at(3));
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
  }
  
  infile.close();
  return point_list;
}


geometry_msgs::Point Gnss_Zmap::gnss_remapping(geometry_msgs::Point pose)
{
  if(point_list.size() == 0)
  {
    cout<<"no tree to search,remapping fail!"<<endl;
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
  if (vectorsize <= 3)
  {
    return point_list[0];
  }
  
  double dist1 = pow(point.x - point_list[0].x, 2) + pow(point.y - point_list[0].y, 2);
  double dist2 = pow(point.x - point_list[vectorsize/2 - 1].x, 2) + pow(point.y - point_list[vectorsize/2 - 1].y, 2);
  double dist3 = pow(point.x - point_list[vectorsize-1].x, 2) + pow(point.y - point_list[vectorsize-1].y, 2);

  if(dist1 + dist2 > dist2 + dist3)
    return Gnss_Zmap::search(std::vector<geometry_msgs::Point>(point_list.begin() + vectorsize/2, point_list.begin() + vectorsize),point);
  else
    return Gnss_Zmap::search(std::vector<geometry_msgs::Point>(point_list.begin(), point_list.begin() + vectorsize/2),point);
}
}



