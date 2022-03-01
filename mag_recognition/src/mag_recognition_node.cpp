/*
 * @Author: your name
 * @Date: 2021-12-23 13:41:04
 * @LastEditTime: 2022-01-10 10:10:38
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /mag_recognition/src/mag_recognition_node.cpp
 */
#include <iostream>
#include "mag_utility.h"

#include "SCManager.h"
#include "hinson_640n/Hinson640nData.h"

bool get_laser = false;

pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
void cloudHandler(const sensor_msgs::PointCloud2::ConstPtr points_data)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*points_data ,*new_cloud);
    std::vector<int> indices;
    removeNaN(*new_cloud, *new_cloud, indices);
    *current_cloud_ptr = *new_cloud;
    get_laser = true;
}

int main(int argc, char **argv)
{
    //log
    google::InitGoogleLogging(argv[0]);
    FLAGS_stop_logging_if_full_disk = true;
    FLAGS_log_dir = WORK_SPACE_PATH + "/log/mag_recognition";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "mag_recognition_node");
    ros::NodeHandle nh;


    //配置文件路径 
    std::string config_file_path = WORK_SPACE_PATH + "/config/config.yaml";
    //std::cout<<"Load test node config path: "<<config_file_path<<std::endl;
    LOG(INFO)<<"init node and config, node name is: mag_recognition_node , config path is: " <<config_file_path;
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    std::string method = "ScanContext";
    std::string method_config_path = WORK_SPACE_PATH + "/config/sc_config.yaml";
    std::string laser_file_path =  WORK_SPACE_PATH + "/data/MagInfo/";
    std::string mag_pose_path =  WORK_SPACE_PATH + "/data/MagInfo/";
    std::string sub_laser_topic = "points_raw";
    std::string pub_current_pose_topic = "recogniton_pose";
    std::string pub_marker_topic = "mag_pose_marker";
    bool need_to_correct = false;

    sub_laser_topic = config_node["SubLaserTopic"].as<std::string>();
    pub_current_pose_topic = config_node["PubMagPoseTopic"].as<std::string>();
    
    ros::Subscriber subLaserCloud;
    //订阅当前点云 用来对照字典来反馈磁钉位置
    subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(sub_laser_topic, 5, cloudHandler);
    //若成功识别 发布mag坐标信息
    ros::Publisher pub_pose = nh.advertise<geometry_msgs::Pose>(pub_current_pose_topic , 10);
    //在rviz上显示 marker
    ros::Publisher pub_marker_array = nh.advertise<visualization_msgs::MarkerArray>(pub_marker_topic, 10);

    method = config_node["RecognMethod"].as<std::string>();
    //是否选择默认读取信息的路径
    bool use_default_path = true;
    use_default_path = config_node["UseDefaultPath"].as<bool>();
    if(!use_default_path){
        laser_file_path = config_node["LaserFilePath"].as<std::string>();
        mag_pose_path = config_node["MagPoseFilePath"].as<std::string>();        
    }
    need_to_correct = config_node["CorrectPose"].as<bool>();

    LaserRecogniton *lr;
    if(method == "ScanContext"){
        LOG(INFO)<<"method : ScanContext";
        method_config_path = WORK_SPACE_PATH + "/config/sc_config.yaml";
        LOG(INFO)<<"load sc config , path : "<<method_config_path;
        lr = new SCManager;
    }else if(method == "M2DP"){
        LOG(INFO)<<"method : M2DP";
        method_config_path = WORK_SPACE_PATH + "/config/m2dp_config.yaml";
        LOG(INFO)<<"load m2dp config , path : "<<method_config_path;

    }else{
        LOG(WARNING) <<"no method called : "<<method <<" , use basic method ScanContext";
        method_config_path = WORK_SPACE_PATH + "/config/sc_config.yaml";
        LOG(INFO)<<"load sc config , path : "<<method_config_path;
        lr = new SCManager;
        
    }

    //获取配置 导入磁钉与激光历史记录数据
    YAML::Node method_config = YAML::LoadFile(method_config_path);
    if(!lr->InitConfig(method_config)){
        LOG(ERROR)<<"method config wrong , return";
        return 0;
    }

    if(!lr->ReadMagFile(mag_pose_path)){
        LOG(ERROR)<<"no mag pose file , return";
        return 0;        
    }

    if(!lr->ReadLaserFile(laser_file_path)){
        LOG(ERROR)<<"no laser file , return";
        return 0;        
    }

    LOG(INFO)<<"success load laser and mag pose file";
    //制作字典
    if(lr->CheckFiles()){
        if(lr->MakeHistoryDir()){
            LOG(INFO)<<"congulation !, make laser dictionary successed ";
        }else{
            LOG(ERROR)<<"make laser dictionary error ";
        }
    }else{
        LOG(ERROR)<<"check files wrong ";
    }


    ros::Rate rate_(10);
    while(ros::ok())
    {
        if(pub_marker_array.getNumSubscribers()>0){
            std::vector<geometry_msgs::Pose> marker_pose = lr->GetRecordMagPose();
            visualization_msgs::MarkerArray mag_ms;
            int pose_size = marker_pose.size();
            for(int i = 0; i<pose_size ; i++){
                visualization_msgs::Marker mag_m;
                mag_m.header.frame_id = "/map";
                mag_m.header.stamp = ros::Time::now();
                mag_m.ns = "mag_marker";
                mag_m.id = i;
                mag_m.type = visualization_msgs::Marker::ARROW;
                // DELETE
                mag_m.action = visualization_msgs::Marker::ADD;
                mag_m.pose = marker_pose[i];
                mag_m.scale.x = 0.2;
                mag_m.scale.y = 0.2;
                mag_m.scale.z = 0.5;
                mag_m.color.r = 200.0;
                mag_m.color.g = 0.0;
                mag_m.color.b = 0.0;
                mag_m.color.a = 1.0;
                mag_ms.markers.push_back(mag_m);  
            }
            pub_marker_array.publish(mag_ms);
        }

        int reco_index = -1;
        geometry_msgs::Pose reco_pose;
        if(get_laser){
        
            pcl::PointCloud<pcl::PointXYZ> new_laser = *current_cloud_ptr;
            lr->SetCurrentLaser(new_laser);
            //单线程 可能会阻塞
            if(lr->RecognLaser()){
                if(need_to_correct)
                    lr->GetCorrectIndexAndPose(reco_index , reco_pose);
                else 
                    lr->GetOriginIndexAndPose(reco_index , reco_pose);
            }
        }



        //识别成功 发布出去
        if(reco_index >= 0){
            pub_pose.publish(reco_pose);
            LOG(INFO) <<"recogition pose , index is : "<<reco_index;
        }
        ros::spinOnce();
        rate_.sleep();
    }

    
    return 0;
}