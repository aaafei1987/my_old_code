/*
 * @Author: your name
 * @Date: 2021-12-23 13:49:56
 * @LastEditTime: 2022-01-05 10:37:27
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AElas
 * @FilePath: /mag_recognition/include/SCManager.h
 */
#include "LaserRecognition.h"
#include "kdtree_vector_of_vectors_adaptor.hpp"



class SCManager: public LaserRecogniton
{
    public:
        //需要kdtree加速搜索 适配
        typedef KDTreeVectorOfVectorsAdaptor<std::vector<std::vector<float>>, float> RingKeyIndex;

        SCManager();
        ~SCManager();

        bool InitConfig(const YAML::Node node);
        
        bool MakeHistoryDir();
        //记录当前点云 
        void SetCurrentLaser(pcl::PointCloud<pcl::PointXYZ> cur_laser);
        //查找 成功就返回true 失败false
        bool RecognLaser();
        //反馈结果 index号与对应位姿（该位姿是未调整 , 原有mag附带的）
        void GetOriginIndexAndPose(int & index , geometry_msgs::Pose & pose);
        //反馈结果 index号与对应位姿（该位姿是调整过的 , 原有mag基础上调整）
        void GetCorrectIndexAndPose(int & index , geometry_msgs::Pose & pose);

    private:

        Eigen::MatrixXf GetScanContext(pcl::PointCloud<pcl::PointXYZ> input_scan);

        std::vector<float> GetRingKing(Eigen::MatrixXf scan_context);

        Eigen::MatrixXf GetSectorKey(const Eigen::MatrixXf &scan_context);

        float GetOrientation(const float &x, const float &y); 

        int GetIndex(const float &value,const float &MAX_VALUE,const int RESOLUTIO);

        //比较两个scan context ，返回评价分数.抄的
        std::pair<int, float> GetScanContextMatch(const Eigen::MatrixXf &target_scan_context, const Eigen::MatrixXf &source_scan_context);
        //循环移动matrix图像（抄的）
        Eigen::MatrixXf CircularShift(const Eigen::MatrixXf &mat, int shift );
        //通过sector key初步获取最适合的移动值 （抄的）
        int GetOptimalShiftUsingSectorKey(const Eigen::MatrixXf &target, const Eigen::MatrixXf &source);
        //比较sc图像 返回得分
        float GetCosineDistance(const Eigen::MatrixXf &target_scan_context, const Eigen::MatrixXf &source_scan_context);


        //整个系统就是在维护这个状态
        struct {
        //点云转换成的二维矩阵 sc字典
        std::vector<Eigen::MatrixXf> sc_matrix;
        //二维数组 1维代表index号 2维代表该index的点云ring
        std::vector<std::vector<float>> ring_key;
        //存储mag的pose
        std::vector<geometry_msgs::Pose> index_pose;

        std::shared_ptr<RingKeyIndex> kd_tree;
    
        }sc_state;

        //返回结果 index yaw
        std::pair<int, float> match_result_;
        //当前激光的sc 与对应的 ringkey
        Eigen::MatrixXf current_matrix_;
        std::vector<float> current_ring_key_;

        //参数
        // a. ROI definition:
        float MAX_RADIUS_;
        float MAX_THETA_;
        // b. resolution:
        int NUM_RINGS_;
        int NUM_SECTORS_; 
        float DEG_PER_SECTOR_;
        // c. ring key indexing interval:
        int INDEXING_INTERVAL_;
        // d. min key frame sequence distance:
        int MIN_KEY_FRAME_SEQ_DISTANCE_;
        // e. num. of nearest-neighbor search candidates:
        int NUM_CANDIDATES_;
        // f. sector key fast alignment search ratio:
        float FAST_ALIGNMENT_SEARCH_RATIO_;
        // g. scan context distance threshold:
        float SCAN_CONTEXT_DISTANCE_THRESH_;

};