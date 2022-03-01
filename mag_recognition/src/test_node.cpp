/*
 * @Author: your name
 * @Date: 2022-01-10 08:53:52
 * @LastEditTime: 2022-01-11 08:40:08
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /mag_recognition/src/test_node.cpp
 */
#include <iostream>
#include "mag_utility.h"
#include "hinson_640n/Hinson640nData.h"

int main(int argc, char **argv)
{
    //log
    google::InitGoogleLogging(argv[0]);
    FLAGS_stop_logging_if_full_disk = true;
    FLAGS_log_dir = WORK_SPACE_PATH + "/log/mag_test";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "mag_test_node");
    ros::NodeHandle nh; 

    //配置文件路径 
    std::string config_file_path = WORK_SPACE_PATH + "/config/record_config.yaml";
    LOG(INFO)<<"init node and config, node name is: mag_test_node , config path is: " <<config_file_path;
    YAML::Node config_node = YAML::LoadFile(config_file_path);  

    std::string pose_topic =  config_node["SubPoseTopic"].as<std::string>();
    std::string mag_topic =  config_node["SubMagTopic"].as<std::string>(); 
    ros::Publisher pose_pub = nh.advertise<RosPoseType>(pose_topic, 10);
    ros::Publisher mag_pub = nh.advertise<hinson_640n::Hinson640nData>(mag_topic, 10);
    
    ros::Rate rate_(10);
    int count = 0;
    double gen = -21.922;
    while(ros::ok()){
        //fake data to test
        RosPoseType tp;
        tp.header.frame_id = "map";
        tp.header.stamp = ros::Time::now();
        tp.pose.pose.position.x = gen;
        tp.pose.pose.position.y = gen;
        tp.pose.pose.position.z = gen; 
        tp.pose.pose.orientation.w = 1.0;
        tp.pose.pose.orientation.x = 0.0;
        tp.pose.pose.orientation.y = 0.0;
        tp.pose.pose.orientation.z = 0.0;
        pose_pub.publish(tp);

        //gen 更新 
        gen += 0.02 ;
        if(gen > 50.0) gen = - 20.0;

        count ++;
        if(count > 200){
            count = 0;
            hinson_640n::Hinson640nData hhp;
            hhp.node_id  = 1;
            hhp.distance_left = 10;
            hhp.distance_middle = 2;
            hhp.distance_right = 10;
            mag_pub.publish(hhp);
        }

        rate_.sleep();
        ros::spinOnce();
    }
}