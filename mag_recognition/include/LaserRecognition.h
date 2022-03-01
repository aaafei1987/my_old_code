/*
 * @Author: your name
 * @Date: 2021-12-23 14:28:49
 * @LastEditTime: 2022-01-10 09:06:07
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /include/LaserRecognition.h
 */
#include <iostream>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/Pose.h>
#include "mag_utility.h"

//作为基类 抽象类 构造函数为空 
//当使用sc m2dp等各种点云识别算法时，都要在该基类上实现
//抽象类提供点云与mag信息的读取，其他具体保存与检测方式需要派生类实现
class LaserRecogniton{
    public:
        LaserRecogniton(){
            have_mag_file_ = false;
            have_laser_file_ = false;
        }
        
        ~LaserRecogniton(){
            record_mag_pose_.clear();
            record_history_laser_.clear();
            all_mag_data_vector_.clear();
            have_mag_file_ = false;
            have_laser_file_ = false;
        }

        bool ReadMagFile(std::string mag_file_path){
            std::string mag_file = mag_file_path + "mag_pose.txt";
            LOG(INFO)<<"load mag_pose.txt , path : "<<mag_file;
            FILE* infile = fopen(mag_file.c_str() , "r");
            if(!infile){
                have_mag_file_ = false;
                LOG(ERROR)<<"mag_pose.txt not exist in "<<mag_file;
                return false;
            }   
            
            for(int i=1 ; i<= 100000; i++){

                double x,y,z,w,qx,qy,qz,unused;
                int node_id,left,middle,right;
                int index_temp = 0;
                fscanf(infile , "%d %lf %lf %lf %lf %lf %lf %lf %d %d %d %d %lf\n" ,&index_temp ,
                    &x,&y,&z,&w,&qx,&qy,&qz,&node_id,&left,&middle,&right,&unused);
                //说明已经读完了 直接跳出 最多读100000行
                if(index_temp != i)break; 
                
                geometry_msgs::Pose mag_info_temp;
                mag_info_temp.position.x = x;
                mag_info_temp.position.y = y;
                mag_info_temp.position.x = z;
                mag_info_temp.orientation.w = w;
                mag_info_temp.orientation.x = qx;
                mag_info_temp.orientation.y = qy;
                mag_info_temp.orientation.z = qz;
                record_mag_pose_.push_back(mag_info_temp);

                MagInfoStruct mag_struct;
                mag_struct.mag_index = i;
                mag_struct.mag_pose.pose_x = x;
                mag_struct.mag_pose.pose_y = y;
                mag_struct.mag_pose.pose_z = z;
                mag_struct.mag_pose.orientation_w = w;
                mag_struct.mag_pose.orientation_x = qx;
                mag_struct.mag_pose.orientation_y = qy;
                mag_struct.mag_pose.orientation_z = qz;
                mag_struct.mag_data.node_id = node_id;
                mag_struct.mag_data.distance_left = left;
                mag_struct.mag_data.distance_middle = middle;
                mag_struct.mag_data.distance_right = right;
                mag_struct.mag_data.expend_unused = unused;
                all_mag_data_vector_.push_back(mag_struct);
                //LOG(INFO)<<"load magpose index: "<<index_temp;
                //LOG(INFO)<<"mag info : " << x<<y<<z<<w<<qx<<qy<<qz;
            }

            fclose(infile);
            LOG(INFO)<<"finish read mag_pose.txt , size is "<<record_mag_pose_.size();
            have_mag_file_ = true;
            return true;
        }

        bool ReadLaserFile(std::string laser_file_path){

            if(!have_mag_file_){
                LOG(ERROR)<<"you should load mag pose first";
                return false;
            }else if(record_mag_pose_.size() == 0){
                LOG(ERROR)<<"mag pose size == 0 , not read laser file";
                return false;                
            }
            
            std::vector<std::string> filenames_vector;
            for(int i=1; i<= record_mag_pose_.size() ; i++){
                std::string filename_temp = laser_file_path + "mag_laser_" + std::to_string(i) + ".pcd";
                filenames_vector.push_back(filename_temp);
            }
            //多校验一个 以防数据不对
            int ext = record_mag_pose_.size() + 1;
            std::string extra_laser_tocheck = laser_file_path + "mag_laser_" + std::to_string(ext) + ".pcd";
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
            if(pcl::io::loadPCDFile<pcl::PointXYZ>(extra_laser_tocheck , *cloud_temp) != -1){
                LOG(ERROR) << "find extra laser pcd : "<<extra_laser_tocheck ;
                LOG(ERROR) << "but mag_pose.txt no this laser index, check mag_pose file format";
                return false;
            }else{
                LOG(INFO)<<"if pcl log no file error , not worry , its check next file exist ";
            }         

            for(int i = 0; i<filenames_vector.size() ; i++){
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
                std::string filename = filenames_vector[i];
                if(pcl::io::loadPCDFile<pcl::PointXYZ>(filename , *cloud_temp) == -1){
                    LOG(ERROR) << "no laser file : "<<filename ;
                    return false;
                }
                std::vector<int> indices;
                removeNaN(*cloud_temp, *cloud_temp, indices);
                record_history_laser_.push_back(*cloud_temp);
            }

            LOG(INFO)<<"finish read laser files , size is: "<<record_history_laser_.size();
            have_laser_file_ = true;
            return true;
        }

        bool CheckFiles()
        {
            if((!have_laser_file_)||(!have_mag_file_))
                return false;

            if(record_mag_pose_.size() != record_history_laser_.size()){
                LOG(ERROR) <<"laser size not equal mag_pose size !!";
                LOG(ERROR) << "laser size: " <<record_history_laser_.size() <<"  mag_pose size: "<<record_mag_pose_.size();
                return false;
            }

            return true;
        }

        std::vector<geometry_msgs::Pose> GetRecordMagPose()
        {
            std::vector<geometry_msgs::Pose> mag_data;
            if(record_mag_pose_.size() == 0){
                mag_data.clear();
                LOG(ERROR)<<"record pose size is 0 , can't get ";
                return mag_data;
            }
            mag_data = record_mag_pose_;
            return mag_data;
        }


        //config 初始化
        virtual bool InitConfig(const YAML::Node node) = 0;
        //需要派生类实现的， 根据读取数据制作历史字典
        virtual bool MakeHistoryDir() = 0;
        //记录当前点云 
        virtual void SetCurrentLaser(pcl::PointCloud<pcl::PointXYZ> cur_laser) = 0;
        //查找 成功就返回true 失败false
        virtual bool RecognLaser() = 0;
        //反馈结果 index号与对应位姿（该位姿是未调整 , 原有mag附带的）
        virtual void GetOriginIndexAndPose(int & index , geometry_msgs::Pose & pose) = 0;
        //反馈结果 index号与对应位姿（该位姿是调整过的 , 原有mag基础上调整）
        virtual void GetCorrectIndexAndPose(int & index , geometry_msgs::Pose & pose) = 0;
        
        

        bool have_mag_file_;
        bool have_laser_file_;

        std::vector<pcl::PointCloud<pcl::PointXYZ>>  record_history_laser_;
        std::vector<geometry_msgs::Pose> record_mag_pose_;
        std::vector<MagInfoStruct> all_mag_data_vector_;
 
        


};