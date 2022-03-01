/*
 * @Author: your name
 * @Date: 2021-09-30 08:59:30
 * @LastEditTime: 2021-11-05 15:10:10
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /auto_parking/src/auto_parking_node.cpp
 */


#include "data_utility.h"
#include "src_path_defination.h"

using namespace Eigen;

bool finish_config_ = false;
bool have_obstacle_ = false; //标志位
//点云障碍区间参数
double laser_crop_dis_ = 1.0; //这个范围以内的激光点不算障碍
double max_obstacle_dis_ = 3.0; //这个范围以外的激光点不算障碍
std::vector<std::pair<double,double>> angle_interval_; //角度区间，从[-pi,pi] , 可以有多个区间
std::vector<int> select_ring_; //选择ring号 一般是最下面的几条线 但嫌麻烦没做 目前是所有都包含


void LaserCallback(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{ 
    if(!finish_config_) return;
    pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZ>());
    sensor_msgs::PointCloud2 cur_c = *laserCloudMsg;
    pcl::moveFromROSMsg(cur_c, *laserCloudIn);
    //LOG(INFO) << "current cloud size : "<<laserCloudIn->size();

    /*
    //检测雷达是否有ring号       
    static int ringFlag = 0;
    if (ringFlag == 0)
    {
        ringFlag = -1;
        for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i){
            if(currentCloudMsg.fields[i].name == "ring"){
                LOG(INFO) <<"ring field have";
                ringFlag = 1;
                break;
            }
        }
        if (ringFlag == -1){
            LOG(ERROR)<<"Point cloud ring channel not available, please configure your point cloud data!";
            ros::shutdown();
        }
    }*/
    
    int cloudSize = laserCloudIn->size();
    for(int i=0 ; i<cloudSize ; i++)
    {
        double dis = sqrt(pow(laserCloudIn->points[i].x ,2)+pow(laserCloudIn->points[i].y ,2));
        if(dis<= laser_crop_dis_ || dis>=max_obstacle_dis_) continue;

        double angle = atan2(laserCloudIn->points[i].x , laserCloudIn->points[i].y);
        int ve_size = angle_interval_.size();
        //区间判断
        
    }
}



int main(int argc, char **argv)
{
    //log
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/log/laser_obstacle_log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "laser_obstacle_node");
    ros::NodeHandle nh;

    //配置文件路径 
    std::string config_file_path = WORK_SPACE_PATH + "/config/laser_obs_config.yaml";
    //std::cout<<"Load test node config path: "<<config_file_path<<std::endl;
    LOG(INFO)<<"init node and config, node name is: laser_obs_config , config path is: " <<config_file_path;
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    std::string laser_topic = "points_raw";
	nh.getParam("laser_topic", laser_topic);
    ros::Subscriber sub_point_cloud = nh.subscribe(laser_topic , 1, LaserCallback);

    ros::Publisher pub_obstacle_cmd = nh.advertise<std_msgs::Int8>("auto_park_obstacle" , 10);
 

    ros::Rate rate_(100);
    while(ros::ok()){

        if(have_obstacle_){
            std_msgs::Int8 flag;
            flag.data = 1;
            pub_obstacle_cmd.publish(flag);
        }
        else{
            std_msgs::Int8 flag;
            flag.data = 0;
            pub_obstacle_cmd.publish(flag);
        }

        ros::spinOnce();
        rate_.sleep();
    }
    return 0;
}


