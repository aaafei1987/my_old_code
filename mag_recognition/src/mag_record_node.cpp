/*
 * @Author: your name
 * @Date: 2021-12-23 13:41:04
 * @LastEditTime: 2022-01-07 11:20:07
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /mag_recognition/src/mag_record_node.cpp
 */
#include <iostream>
#include "mag_utility.h"
#include "hinson_640n/Hinson640nData.h"


bool get_laser = false;
bool get_pose = false;
bool get_mag_info = false;

double last_pose_x = -999.0;
double last_pose_y = -999.0;

const double DIS_MAG_DETECTED = 0.2; //前后间隔大于这个距离的磁钉才进行录入

std::vector<sensor_msgs::PointCloud2> laser_vector;
std::vector<RosPoseType> pose_vector;
std::vector<hinson_640n::Hinson640nData> mag_info_vector;
std::vector<MagInfoStruct> all_record_info;

void cloudHandler(const sensor_msgs::PointCloud2 points_data)
{
    if(laser_vector.size()>=10)
        laser_vector.clear();
    laser_vector.push_back(points_data);
    get_laser = true;
}

void poseHandler(const RosPoseType pose_info)
{
    if(pose_vector.size()>=15)
        pose_vector.clear();
    pose_vector.push_back(pose_info);
    get_pose = true;
}

void magInfoHandler(const hinson_640n::Hinson640nData mag_data)
{
    LOG(INFO)<<"receive a mag data";
    if(mag_info_vector.size()>=3)
        mag_info_vector.clear();
    
    mag_info_vector.push_back(mag_data);
    get_mag_info = true;
}

void ResetGetFlag()
{
    get_laser = false;
    get_pose = false;
    get_mag_info = false;
    laser_vector.clear();
    pose_vector.clear();
    mag_info_vector.clear();
}

bool CheckFlag()
{
    if(get_laser&&get_pose&&get_mag_info)
    {
        RosPoseType new_pose = pose_vector.back();
        double dis_now_last = SqrtDistance(new_pose.pose.pose.position.x , new_pose.pose.pose.position.y , last_pose_x , last_pose_y);
        if(dis_now_last > DIS_MAG_DETECTED){
            LOG(INFO)<<"detected a mag , record it";
            return true;
        }
        else{
            LOG(INFO)<<"detected a mag , but too close , dis is: " << dis_now_last;
            return false;
        }
    }
    else
        return false;
}

bool RecordNewLaser(std::string laser_filename){

    sensor_msgs::PointCloud2 last_laser = laser_vector.back();
    pcl::PointCloud<pcl::PointXYZ>  trans_cloud;
    pcl::fromROSMsg(last_laser , trans_cloud);
    pcl::io::savePCDFileBinary(laser_filename ,trans_cloud);
    LOG(INFO)<<"record a laser at "<<laser_filename;
    
    //记录此时的pose与mag_data
    RosPoseType new_pose = pose_vector.back();
    hinson_640n::Hinson640nData new_mag_data = mag_info_vector.back();
    MagInfoStruct now_mag_info;
    now_mag_info.mag_pose.pose_x = new_pose.pose.pose.position.x;
    last_pose_x = new_pose.pose.pose.position.x;
    now_mag_info.mag_pose.pose_y = new_pose.pose.pose.position.y;
    last_pose_y =  new_pose.pose.pose.position.y;
    
    now_mag_info.mag_pose.pose_z = new_pose.pose.pose.position.z;
    now_mag_info.mag_pose.orientation_w = new_pose.pose.pose.orientation.w;
    now_mag_info.mag_pose.orientation_x = new_pose.pose.pose.orientation.x;
    now_mag_info.mag_pose.orientation_y = new_pose.pose.pose.orientation.y;
    now_mag_info.mag_pose.orientation_z = new_pose.pose.pose.orientation.z;
    now_mag_info.mag_data.node_id = new_mag_data.node_id;
    now_mag_info.mag_data.distance_left = new_mag_data.distance_left;
    now_mag_info.mag_data.distance_middle = new_mag_data.distance_middle;
    now_mag_info.mag_data.distance_right = new_mag_data.distance_right;
    now_mag_info.mag_data.expend_unused = 0;
    all_record_info.push_back(now_mag_info);
    return true;
}

bool RecordMagInfo(std::string mag_filename)
{
    FILE* of = fopen(mag_filename.c_str() , "w");
    int nums = all_record_info.size();
    for(int i=0;i<nums;i++){
        //往文件输入 index ndt(x,y,z qw qx qy qz) mag(nodeid , left , middle , right ,unused)
        fprintf(of , "%d  %12.10f  %12.10f  %12.10f  %12.10f  %12.10f  %12.10f  %12.10f  %d %d %d %d %12.10f\n" , 
          i+1 ,
          all_record_info[i].mag_pose.pose_x , all_record_info[i].mag_pose.pose_y , 
          all_record_info[i].mag_pose.pose_z , all_record_info[i].mag_pose.orientation_w,
          all_record_info[i].mag_pose.orientation_x , all_record_info[i].mag_pose.orientation_y ,
          all_record_info[i].mag_pose.orientation_z , all_record_info[i].mag_data.node_id,
          all_record_info[i].mag_data.distance_left, all_record_info[i].mag_data.distance_middle,
          all_record_info[i].mag_data.distance_right, all_record_info[i].mag_data.expend_unused );        
    }
    fclose(of);
    LOG(INFO)<<"save mag_info , path is: "<<mag_filename;
    return true;
}

int main(int argc, char **argv)
{
    //log
    google::InitGoogleLogging(argv[0]);
    FLAGS_stop_logging_if_full_disk = true;
    FLAGS_log_dir = WORK_SPACE_PATH + "/log/mag_record";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "mag_record_node");
    ros::NodeHandle nh;


    //配置文件路径 
    std::string config_file_path = WORK_SPACE_PATH + "/config/record_config.yaml";
    //std::cout<<"Load test node config path: "<<config_file_path<<std::endl;
    LOG(INFO)<<"init node and config, node name is: mag_record_node , config path is: " <<config_file_path;
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    ros::Subscriber subLaserCloud;
    ros::Subscriber subPose;
    ros::Subscriber subMag;
    
    std::string laser_record_path =  WORK_SPACE_PATH + "/record_data/MagInfo/";
    std::string mag_pose_record_path =  WORK_SPACE_PATH + "/record_data/MagInfo/";
    bool use_default_path = true;
    use_default_path = config_node["UseDefaultPath"].as<bool>();
    if(!use_default_path){
        laser_record_path = config_node["RecordLaserPath"].as<std::string>();
        mag_pose_record_path = config_node["RecordMagInfoPath"].as<std::string>();        
    }
    LOG(INFO)<<"record laser path is: "<<laser_record_path;
    LOG(INFO)<<"record mag_info path is: "<<mag_pose_record_path;

    std::string sub_laser_topic = "points_raw";
    std::string sub_pose_topic = "ndt_pose";
    std::string sub_mag_topic = "hinson_640n/magnetic_data";
   

    sub_laser_topic = config_node["SubLaserTopic"].as<std::string>();
    sub_pose_topic = config_node["SubPoseTopic"].as<std::string>();
    sub_mag_topic = config_node["SubMagTopic"].as<std::string>();
    LOG(INFO)<<"run this record program , you need topic ---> "<<sub_laser_topic<<" || "<<sub_pose_topic<<" || "<<sub_mag_topic;
    
    //订阅当前点云和位置信息 用来记录点云信息和定位信息 ; 订阅magpose信息进行触发并记录
    subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(sub_laser_topic, 5, cloudHandler);

    subPose = nh.subscribe<RosPoseType>(sub_pose_topic, 5, poseHandler); 

    subMag = nh.subscribe<hinson_640n::Hinson640nData>(sub_mag_topic , 5 , magInfoHandler);

    
    //记录此时laser的名字index 从1开始
    static int laser_name_index = 1;
    ros::Rate rate_(50);
    while(ros::ok())
    {
        //清空文件夹与文件 没有的话就会创建
        static bool clean_flag = false;
        if(!clean_flag){
            LOG(INFO) << "clear laser/mag_info path files! ";
            int unused = system((std::string("exec rm -r ") + laser_record_path).c_str());
            unused = system((std::string("mkdir -p ") + laser_record_path).c_str()); 
            LOG(INFO) << "clear laser path : " <<laser_record_path;
            if(mag_pose_record_path != laser_record_path){
                unused = system((std::string("exec rm -r ") + mag_pose_record_path).c_str());
                unused = system((std::string("mkdir -p ") + mag_pose_record_path).c_str());  
                LOG(INFO) << "clear mag_info path : " <<mag_pose_record_path;               
            }   
            clean_flag = true;
        }

        if(CheckFlag()){
            std::string laser_file_name = laser_record_path + "mag_laser_" + std::to_string(laser_name_index) + ".pcd";
            if(!RecordNewLaser(laser_file_name)){
                LOG(ERROR)<<"save laser failed , path is: "<<laser_file_name;
                return 0;
            }
            ResetGetFlag();
            laser_name_index ++;
        }
        ros::spinOnce();
        rate_.sleep();
    }

    std::string mag_info_file_name = mag_pose_record_path + "mag_pose.txt";
    if(!RecordMagInfo(mag_info_file_name))
        LOG(ERROR)<<"record maginfo error , path is: "<<mag_info_file_name;
    return 0;
}