/*
 * @Author: your name
 * @Date: 2021-09-30 08:59:30
 * @LastEditTime: 2021-11-02 16:50:56
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /auto_parking/src/generate_path.h
 */

#ifndef GENERATE_PATH_H_
#define GENERATE_PATH_H_

#include "data_utility.h"

class CreatPath{

    public:
        CreatPath();
        bool InitConfig(const YAML::Node yaml_node);
        //制作倒车轨迹  给出轨迹起点与终点和终点走的直线距离，会同时生成倒车序列与出车序列（只是顺序相反而已）
        //旋转半径在内部计算，不会小于最小回转半径  start_ori 与 end_ori 都是 (-pi , pi] 与atan2一致
        bool CalPath(Pose2d start , double start_ori , Pose2d end , double end_ori , double line_distance);
        bool GetParkingPath(std::vector<Pose2d>& parkingpath);
        bool GetOutingPath(std::vector<Pose2d>& outingpath);
        //清除存储的path
        bool ResetPathData();

     private:
    

        double min_corner_r; //最小转弯半径
        double begin_line_distance; //在start ori 前延长一段线
        double overshoot_line; //pure pursuit跟踪到终点，会有过调问题，额外加一段用来路径跟踪，但是不行驶
        std::vector<Pose2d> parking_path_;
        std::vector<Pose2d> outing_path_;
        Pose2d overshoot_begin_pose_;
        Pose2d overshoot_end_pose_;


};

#endif