#include "utility.h"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)


class Fusion : public ParamServer
{
public:

    std::mutex mtx;

    //话题订阅
    ros::Subscriber subImu;         //imu位姿
    ros::Subscriber subPoint;       //gps位置
    ros::Subscriber subPose_lidar;  //激光匹配位姿
    ros::Subscriber subPose_gnss;   //RTK位姿
    ros::Subscriber subOdom;        //里程计
    ros::Subscriber subInitial;     //重发布位置

    //话题发布
    ros::Publisher pubImuOdometry;
    ros::Publisher pubPose;
    ros::Publisher pubSlamPose;
    ros::Publisher pubInitial;


    //误差模型
    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
    gtsam::noiseModel::Diagonal::shared_ptr ncorrectionNoise;
    gtsam::noiseModel::Diagonal::shared_ptr odomNoise;
    gtsam::noiseModel::Diagonal::shared_ptr gcorrectionNoise;
    gtsam::noiseModel::Diagonal::shared_ptr gcorrectionNoise_2;
    gtsam::Vector noiseModelBetweenBias;


    //位姿结构体
    Eigen::Affine3f pastposeAffine;
    Eigen::Affine3f latestposeAffine;
    Eigen::Affine3f rtkAffine;
    Eigen::Affine3f latestRtkAffine;
    Eigen::Affine3f gpsAffine;
    Eigen::Affine3f latestgpsAffine;
    Eigen::MatrixXd poseCovariance;

    geometry_msgs::PoseWithCovarianceStamped pose_slam;


    //消息时间
    double pastposeTime;
    double poseTime = -1;
    double pointTime = -1;
    double pastRtkTime;
    double pastGpsTime;
    double lastImuT_opt = -1;
    double currentUpdateTime = -1;


    //标志量
    bool systemInitialized = false; //系统初始化
    bool updated = false;           //融合位置更新
    bool enablePub = false;         //允许发布标志位： 在接收到initialpose 或系统首次定位未稳定过程中不发布融合位姿
    bool selfpub = false;           //自身发布的initialpose进行ndt修正时不打断重置融合进程
    bool doneFirstOpt = false;      //初始化完成


    //imu积分器
    gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_;


    //消息队列
    deque<nav_msgs::Odometry> imuOdomQueue;
    deque<geometry_msgs::PoseStamped> ndtPoseQue;
    deque<geometry_msgs::PoseStamped> rtkPoseQue;
    deque<geometry_msgs::PoseStamped> pointQue;
    
    std::deque<sensor_msgs::Imu> imuQueOpt;

    //节点状态量
    gtsam::Pose3 prevPose_;
    gtsam::Vector3 prevVel_;
    gtsam::Vector3 odomVel_;
    gtsam::NavState prevState_;
    gtsam::imuBias::ConstantBias prevBias_;

    gtsam::NavState prevStateOdom;
    gtsam::imuBias::ConstantBias prevBiasOdom;

    //优化器与相关变量
    gtsam::ISAM2 optimizer;
    gtsam::NonlinearFactorGraph graphFactors;
    gtsam::Values graphValues;

    //运算延时
    const double delta_t = 0;

    //节点序号
    int key = -1;

    //坐标变换
    gtsam::Pose3 imu2Lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(-extTrans.x(), -extTrans.y(), -extTrans.z()));
    gtsam::Pose3 lidar2Imu = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(extTrans.x(), extTrans.y(), extTrans.z()));

    Fusion()
    {
        ROS_INFO("enter fusion inited");
        //rostopic 回调定义
        subImu      = nh.subscribe<sensor_msgs::Imu>  (imuTopic, 2000, &Fusion::imuHandler, this, ros::TransportHints().tcpNoDelay());
        subPose_lidar = nh.subscribe<geometry_msgs::PoseStamped>("ndt_pose", 5,    &Fusion::poseHandler, this, ros::TransportHints().tcpNoDelay());
        subPose_gnss = nh.subscribe<geometry_msgs::PoseStamped>("gnss_pose", 5,    &Fusion::poseHandler, this, ros::TransportHints().tcpNoDelay());
        subPoint = nh.subscribe<geometry_msgs::PointStamped>("gps_point", 5,    &Fusion::pointHandler, this, ros::TransportHints().tcpNoDelay());
        subOdom = nh.subscribe<nav_msgs::Odometry>("odom", 5,    &Fusion::odomHandler, this, ros::TransportHints().tcpNoDelay());
        subInitial = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 5, &Fusion::initialHandler, this, ros::TransportHints().tcpNoDelay());
        ROS_INFO("callback inited");
        //rostopic 发布定义
        pubPose= nh.advertise<geometry_msgs::PoseStamped> ("isam_pose", 10);
        pubSlamPose= nh.advertise<geometry_msgs::PoseWithCovarianceStamped> ("slam_pose", 10);
        pubInitial= nh.advertise<geometry_msgs::PoseWithCovarianceStamped> ("initialpose", 10);
        pubImuOdometry = nh.advertise<nav_msgs::Odometry> (odomTopic+"_incremental", 2000);

        ROS_INFO("publisher inited");
        //用于imu预积分的一些变量 这块是gtsam涉及的比较多
        //噪声模型
        //白噪声定义（底噪+重力）
        boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imuGravity);
        p->accelerometerCovariance  = gtsam::Matrix33::Identity(3,3) * pow(imuAccNoise, 2); // acc white noise in continuous
        p->gyroscopeCovariance      = gtsam::Matrix33::Identity(3,3) * pow(imuGyrNoise, 2); // gyro white noise in continuous
        p->integrationCovariance    = gtsam::Matrix33::Identity(3,3) * pow(1e-4, 2); // error committed in integrating position from velocities
        gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());; // assume zero initial bias

        //噪声定义
        priorPoseNoise  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m
        priorVelNoise   = gtsam::noiseModel::Isotropic::Sigma(3, 1e-1); // m/s
        priorBiasNoise  = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3); // 1e-2 ~ 1e-3 seems to be good
        ncorrectionNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished()); // rad,rad,rad,m, m, m
        odomNoise= gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished()); // rad,rad,rad,m, m, m
        gcorrectionNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.1, 0.1, 0.1, 1, 1, 1).finished()); // rad,rad,rad,m, m, m
        gcorrectionNoise_2 = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.1, 0.1, 0.1, 1, 1, 1).finished()); // rad,rad,rad,m, m, m
        noiseModelBetweenBias = (gtsam::Vector(6) << imuAccBiasN, imuAccBiasN, imuAccBiasN, imuGyrBiasN, imuGyrBiasN, imuGyrBiasN).finished();
        
        //初始化imu积分器
        imuIntegratorOpt_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for optimization        
    }

    ~Fusion()
    {
        ROS_INFO("deinit");
    }

    //重置优化器
    void resetOptimization()
    {
        gtsam::ISAM2Params optParameters;
        optParameters.relinearizeThreshold = 0.01;
        optParameters.relinearizeSkip = 1;
        optimizer = gtsam::ISAM2(optParameters);

        gtsam::NonlinearFactorGraph newGraphFactors;
        graphFactors = newGraphFactors;

        gtsam::Values NewGraphValues;
        graphValues = NewGraphValues;
    }

    void resetParams()
    {
        doneFirstOpt = false;
        systemInitialized = false;
    }

    //将odom消息格式转换为pcl格式
    Eigen::Affine3f odom2affine(nav_msgs::Odometry odom)
    {
        double x, y, z, roll, pitch, yaw;
        x = odom.pose.pose.position.x;
        y = odom.pose.pose.position.y;
        z = odom.pose.pose.position.z;
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(odom.pose.pose.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        return pcl::getTransformation(x, y, z, roll, pitch, yaw);
    }

    //将pose消息格式转换为pcl格式
    Eigen::Affine3f pose2affine(geometry_msgs::PoseStamped pose)
    {
        double x, y, z, roll, pitch, yaw;
        x = (double)pose.pose.position.x;
        y = (double)pose.pose.position.y;
        z = (double)pose.pose.position.z;
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(pose.pose.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        return pcl::getTransformation(x, y, z, roll, pitch, yaw);
    }

    //将point消息格式转换为pcl格式
    Eigen::Affine3f point2affine(geometry_msgs::PoseStamped pose)
    {
        double x, y, z, roll, pitch, yaw;
        x = pose.pose.position.x;
        y = pose.pose.position.y;
        z = pose.pose.position.z;
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(imuQueOpt.back().orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        return pcl::getTransformation(x, y, z, roll, pitch, yaw);
    }


    //位姿更新主循环流程
    void updateThread()
    {
        ros::Rate rate(updateFrequency);
        while (ros::ok())
        {
            rate.sleep();
            if(!ndtPoseQue.empty() || !rtkPoseQue.empty())
            {
                updatePose(); //融合位姿更新
            }
            updateOdom();   //根据imu与里程计数据更新位姿
            updated = false;
        }
    }

    void updateOdom()
    {
        std::lock_guard<std::mutex> lock(mtx);
        //std::lock_guard<std::mutex> lock(mtx);

        //从积分器中获取当前位姿
        gtsam::NavState currentState = imuIntegratorOpt_->predict(prevStateOdom, prevBiasOdom);

        //检测是否已初始化完毕，发布位姿
        if(enablePub == true && systemInitialized == true)
        {

            // publish odometry
            geometry_msgs::PoseStamped pose_output;
            pose_output.header.stamp = ros::Time::now();
            pose_output.header.frame_id = "map";
            pose_slam.header.stamp = ros::Time::now();
            pose_slam.header.frame_id = "map";

            // transform imu pose to ldiar
            gtsam::Pose3 outPose = gtsam::Pose3(currentState.quaternion(), currentState.position());
            gtsam::Point3 p = outPose.translation();
            gtsam::Quaternion q = outPose.rotation().toQuaternion();
            pose_output.pose.position.x = p.x();
            pose_output.pose.position.y = p.y();
            pose_output.pose.position.z = p.z();
            pose_output.pose.orientation.w = q.w();
            pose_output.pose.orientation.x = q.x();
            pose_output.pose.orientation.y = q.y();
            pose_output.pose.orientation.z = q.z();


            pose_slam.pose.pose.position.x = p.x();
            pose_slam.pose.pose.position.y = p.y();
            pose_slam.pose.pose.position.z = p.z();
            pose_slam.pose.pose.orientation.w = q.w();
            pose_slam.pose.pose.orientation.x = q.x();
            pose_slam.pose.pose.orientation.y = q.y();
            pose_slam.pose.pose.orientation.z = q.z();

            pubPose.publish(pose_output);
            // poseCovariance= optimizer.marginalCovariance(X(key - 1));
            // for(int i = 1; i < 6; i++)
            // {
            //     for(int j = 1;j < 6; j++)
            //     {
            //         // cout << "x1 covariance:\n" << optimizer.marginalCovariance(X(key - 1)) << endl;
            //         // cout << "i,j,cov: "<< i << j << poseCovariance(i,j);
            //         // pose_slam.pose.covariance[i*6+j] = poseCovariance(i,j);
            //     }
            // } 

            pubSlamPose.publish(pose_slam);

        }
    }

    void updatePose()
    {
        std::lock_guard<std::mutex> lock(mtx);

        // make sure we have imu data to integrate
        //保证有imuodom数据  两个更新函数是互有联系的  在odom更新要先确认完成一次pose更新，（即初始位置初始化）才往下执行
        if (imuOdomQueue.empty())
        {
            ROS_INFO("odomque emptty");
            return;
        }

        // make sure we have imu data to integrate
        if (imuQueOpt.empty())
        {
            ROS_INFO("imuQueOpt emptty");
            return;
        }

        if(ndtPoseQue.size() <= 3 && rtkPoseQue.size() < 10 && pointQue.size() < 3)
        {
            return;
        }


/////////////

        //记录最新的imu更新时间
        currentUpdateTime = ROS_TIME(&imuOdomQueue.back());


        //仅保留最大30个节点，避免运行时间过长导致节点数量溢出。当数量达到30个时以当时位姿与协方差初始化优化系统
        // reset graph for speed
        if (key == 30)
        {
            // get updated noise before reset
            gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(X(key-1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedVelNoise  = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(V(key-1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedBiasNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(B(key-1)));
            // reset graph
            resetOptimization();
            // add pose
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, updatedPoseNoise);
            graphFactors.add(priorPose);
            // add velocity
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, updatedVelNoise);
            graphFactors.add(priorVel);
            // add bias
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, updatedBiasNoise);
            graphFactors.add(priorBias);
            // add values
            graphValues.insert(X(0), prevPose_);
            graphValues.insert(V(0), prevVel_);
            graphValues.insert(B(0), prevBias_);
            // optimize once
            optimizer.update(graphFactors, graphValues);
            graphFactors.resize(0);
            graphValues.clear();

            key = 1;
            enablePub = true;
        }


        // 1. integrate imu data and optimize
        // 如果有未参与运算的imu数据，累计积分，并清空过时的imu数据。
        while (!imuQueOpt.empty())
        {
            // pop and integrate imu data that is between two optimizations
            sensor_msgs::Imu *thisImu = &imuQueOpt.front();
            double imuTime = ROS_TIME(thisImu);
            if (imuTime < currentUpdateTime - delta_t)
            {
                double dt = (lastImuT_opt < 0 || imuTime < lastImuT_opt) ? (1.0 / 500.0) : (0.01);      //因xsens imu时间戳并不连续，按照数据频率使用固定时间间隔进行运算
                // double dt = (lastImuT_opt < 0 || imuTime < lastImuT_opt) ? (1.0 / 500.0) : (imuTime - lastImuT_opt);   //正常imu数据时间处理方式

                // cout << "dt:" << dt << endl;
                // cout << "acceleration:" << thisImu->linear_acceleration.x << "," << thisImu->linear_acceleration.y << ", " <<thisImu->linear_acceleration.z  << endl;
                // cout << "angular_velocity:" << thisImu->angular_velocity.x << "," << thisImu->angular_velocity.y << ", " <<thisImu->angular_velocity.z  << endl;
                // cout << "``````````"  << endl;
                // cout << ".........."  << endl;
                imuIntegratorOpt_->integrateMeasurement(
                        gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                        gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
                
                lastImuT_opt = imuTime;
                imuQueOpt.pop_front();
            }
            else
                break;
        }

        //激光匹配位姿优化
        if(ndtPoseQue.size() > 3)
        {
            //获取当前最新位姿latestposeAffine与数帧前位姿pastposeAffine，
            //通过查找imu积分里程计位姿队列获取这数帧间的位姿变换ndtAffineIncre
            //将此变换叠加到数帧前位姿pastposeAffine上，其结果ndtAffineLast与最新latestposeAffine之间的变换差值ndtAffineDiff作为定位信息的误差衡量
            pastposeAffine = pose2affine(ndtPoseQue.front());
            latestposeAffine = pose2affine(ndtPoseQue.back());
            pastposeTime = ROS_TIME(&ndtPoseQue.front());
            poseTime = ROS_TIME(&ndtPoseQue.back());

            //cout << "poseTime - pastposeTime:"<< poseTime - pastposeTime << endl;
            //利用imuodom队列首尾之间的增量式变换获得最终里程计仿射矩阵S
            Eigen::Affine3f ndtAffineFront = odom2affine(imuOdomQueue.front());
            for(int i = 0; i < imuOdomQueue.size(); i++)
            {
                if(ROS_TIME(&imuOdomQueue.at(i)) > pastposeTime)
                {
                    ndtAffineFront = odom2affine(imuOdomQueue.at(i));
                    break;
                }
            }
            Eigen::Affine3f ndtAffineBack = odom2affine(imuOdomQueue.back());
            Eigen::Affine3f ndtAffineIncre = ndtAffineFront.inverse() * ndtAffineBack;
            Eigen::Affine3f ndtAffineLast = pastposeAffine * ndtAffineIncre;
            Eigen::Affine3f ndtAffineDiff = ndtAffineLast.inverse() * latestposeAffine;
            float x, y, z, roll, pitch, yaw;
            pcl::getTranslationAndEulerAngles(latestposeAffine, x, y, z, roll, pitch, yaw);
            float dx, dy, dz, droll, dpitch, dyaw;
            pcl::getTranslationAndEulerAngles(ndtAffineDiff, dx, dy, dz, droll, dpitch, dyaw);

            //得到雷达的位姿 后续用到 比较关键的一个量
            gtsam::Pose3 lidarPose = gtsam::Pose3(gtsam::Rot3::Ypr(yaw,pitch,roll), gtsam::Point3(x, y, z));
            ncorrectionNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.01 + abs(droll), 0.01 + abs(dpitch), 0.01 + abs(dyaw), 0.01 + pow(dx, 2), 0.01 + pow(dy, 2), 0.01 + pow(dz, 2)).finished()); // rad,rad,rad,m, m, m
            // 0. initialize system
            //只执行一次 初始化系统
            if (systemInitialized == false)
            {
                resetOptimization();
                cout << "enter system initial" << endl;
                // pop old IMU message
                //推出相对较旧的imu消息 保证imu与odometry消息时间同步  因为imu是高频数据所以这是必要的
                while (!imuQueOpt.empty())
                {
                    if (ROS_TIME(&imuQueOpt.front()) < currentUpdateTime - delta_t)
                    {
                        lastImuT_opt = ROS_TIME(&imuQueOpt.front());
                        imuQueOpt.pop_front();
                    }
                    else
                        break;
                }
                // initial pose
                //由激光里程计消息提供位姿  并转到imu坐标系下
                prevPose_ = lidarPose.compose(lidar2Imu);
                //PriorFactor 概念可看gtsam  包括了位姿 速度  bias 
                //加入PriorFactor在图优化中基本都是必需的前提
                gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
                graphFactors.add(priorPose);
                // initial velocity
                prevVel_ = gtsam::Vector3(0, 0, 0);
                gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, priorVelNoise);
                graphFactors.add(priorVel);
                // initial bias
                prevBias_ = gtsam::imuBias::ConstantBias();
                gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise);
                graphFactors.add(priorBias);

                //除了因子外 还要有节点value
                // add values
                graphValues.insert(X(0), prevPose_);
                graphValues.insert(V(0), prevVel_);
                graphValues.insert(B(0), prevBias_);
                // optimize once
                optimizer.update(graphFactors, graphValues);
                graphFactors.resize(0);
                graphValues.clear();

                imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

                prevState_ = gtsam::NavState(prevPose_, prevVel_);
                prevStateOdom = gtsam::NavState(prevPose_, prevVel_);
                prevBiasOdom = prevBias_;
                imuQueOpt.clear();
                imuOdomQueue.clear();

                key = 1;
                systemInitialized = true;
                ROS_INFO("systemInitialized");
                return;
            }

            //正常运行流程，将节点插入优化系统
            gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), lidarPose, ncorrectionNoise);
            graphFactors.add(pose_factor);
            // ROS_INFO("update ndtpose");
            ndtPoseQue.pop_front();
            updated = true;
        }
        else if(rtkPoseQue.size() < 10 && pointQue.size() < 3)//如果消息数量不足以组成队列，直接退出
        {
                return;
        }

        //gnss。rtk位姿优化流程，与激光方式基本相同，不重复赘述。
        if(rtkPoseQue.size() >= 10)
        {
            pastRtkTime = ROS_TIME(&rtkPoseQue.front());
            poseTime = ROS_TIME(&rtkPoseQue.front());
            rtkAffine = pose2affine(rtkPoseQue.front());
            latestRtkAffine = pose2affine(rtkPoseQue.back());


            Eigen::Affine3f rtkAffineFront = odom2affine(imuOdomQueue.front());
            for(int i = 0; i < imuOdomQueue.size(); i++)
            {
                if(ROS_TIME(&imuOdomQueue.at(i)) > pastRtkTime)
                {
                    rtkAffineFront = odom2affine(imuOdomQueue.at(i));
                    break;
                }
            }
            //利用rtk队列首尾之间的增量式变换获得最终里程计仿射矩阵
            Eigen::Affine3f rtkAffineBack = odom2affine(imuOdomQueue.back());
            Eigen::Affine3f rtkAffineIncre = rtkAffineFront.inverse() * rtkAffineBack;
            Eigen::Affine3f rtkAffineLast = rtkAffine * rtkAffineIncre;
            Eigen::Affine3f rtkAffineDiff = rtkAffineLast.inverse() * latestRtkAffine;

            float x, y, z, roll, pitch, yaw;
            pcl::getTranslationAndEulerAngles(rtkAffineLast, x, y, z, roll, pitch, yaw);
            float dx, dy, dz, droll, dpitch, dyaw;
            pcl::getTranslationAndEulerAngles(rtkAffineDiff, dx, dy, dz, droll, dpitch, dyaw);
            float combinenoise = dx * dx + dy * dy + dz * dz + droll * droll + dpitch * dpitch + dyaw * dyaw;
            gtsam::Pose3 rtkPose = gtsam::Pose3(gtsam::Rot3::Ypr(yaw,pitch,roll), gtsam::Point3(x, y, z));
            gcorrectionNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.15, 0.15, 0.15, 1.0, 1.0, 1.0).finished()); // rad,rad,rad,m, m, m
            gcorrectionNoise_2 = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1.5, 1.5, 1.5, 10.0, 10.0, 10.0).finished()); // rad,rad,rad,m, m, m
            // 0. initialize system
            //只执行一次 初始化系统
            if (systemInitialized == false)
            {
                resetOptimization();

                // pop old IMU message
                //推出相对较旧的imu消息 保证imu与odometry消息时间同步  因为imu是高频数据所以这是必要的
                while (!imuQueOpt.empty())
                {
                    if (ROS_TIME(&imuQueOpt.front()) < currentUpdateTime - delta_t)
                    {
                        lastImuT_opt = ROS_TIME(&imuQueOpt.front());
                        imuQueOpt.pop_front();
                    }
                    else
                        break;
                }
                // initial pose
                //由gps消息提供位姿  并转到imu坐标系下
                prevPose_ = rtkPose.compose(lidar2Imu);
                //PriorFactor 概念可看gtsam  包括了位姿 速度  bias 
                //加入PriorFactor在图优化中基本都是必需的前提
                gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
                graphFactors.add(priorPose);
                // initial velocity
                prevVel_ = gtsam::Vector3(0, 0, 0);
                gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, priorVelNoise);
                graphFactors.add(priorVel);
                // initial bias
                prevBias_ = gtsam::imuBias::ConstantBias();
                gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise);
                graphFactors.add(priorBias);

                //除了因子外 还要有节点value
                // add values
                graphValues.insert(X(0), prevPose_);
                graphValues.insert(V(0), prevVel_);
                graphValues.insert(B(0), prevBias_);
                // optimize once
                optimizer.update(graphFactors, graphValues);
                graphFactors.resize(0);
                graphValues.clear();

                imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

                prevState_ = gtsam::NavState(prevPose_, prevVel_);
                prevStateOdom = gtsam::NavState(prevPose_, prevVel_);
                prevBiasOdom = prevBias_;
                imuQueOpt.clear();
                imuOdomQueue.clear();
                
                key = 1;
                systemInitialized = true;
                updated = true;
                ROS_INFO("systemInitialized");
                return;
            }
            gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), rtkPose, (combinenoise < 10)? gcorrectionNoise : gcorrectionNoise_2);
            graphFactors.add(pose_factor);
            rtkPoseQue.pop_front();
            ROS_INFO("update rtkpose");
        }

        //单gps位置优化，由于gps仅具备定位信息，即 (x,y)信息，角度信息直接以一个非常大的协方差输入系统
        else if (pointQue.size() >= 3)
        {
            pointQue.pop_front();

            latestgpsAffine = point2affine(pointQue.back());
            float x, y, z, roll, pitch, yaw;
            pcl::getTranslationAndEulerAngles(latestgpsAffine, x, y, z, roll, pitch, yaw);

            gtsam::Pose3 gpsPose = gtsam::Pose3(gtsam::Rot3::Ypr(yaw,pitch,roll), gtsam::Point3(x, y, z));
            gcorrectionNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 10000.0, 10000.0, 10000.0, 1.0, 1.0, 1.0).finished()); // rad,rad,rad,m, m, m
            // 0. initialize system
            //只执行一次 初始化系统
            if (systemInitialized == false)
            {
                resetOptimization();

                // pop old IMU message
                //推出相对较旧的imu消息 保证imu与odometry消息时间同步  因为imu是高频数据所以这是必要的
                while (!imuQueOpt.empty())
                {
                    if (ROS_TIME(&imuQueOpt.front()) < currentUpdateTime - delta_t)
                    {
                        lastImuT_opt = ROS_TIME(&imuQueOpt.front());
                        imuQueOpt.pop_front();
                    }
                    else
                        break;
                }
                // initial pose
                //由激光里程计消息提供位姿  并转到imu坐标系下
                prevPose_ = gpsPose.compose(lidar2Imu);
                //PriorFactor 概念可看gtsam  包括了位姿 速度  bias 
                //加入PriorFactor在图优化中基本都是必需的前提
                gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
                graphFactors.add(priorPose);
                // initial velocity
                prevVel_ = gtsam::Vector3(0, 0, 0);
                gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, priorVelNoise);
                graphFactors.add(priorVel);
                // initial bias
                prevBias_ = gtsam::imuBias::ConstantBias();
                gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise);
                graphFactors.add(priorBias);

                //除了因子外 还要有节点value
                // add values
                graphValues.insert(X(0), prevPose_);
                graphValues.insert(V(0), prevVel_);
                graphValues.insert(B(0), prevBias_);
                // optimize once
                optimizer.update(graphFactors, graphValues);
                graphFactors.resize(0);
                graphValues.clear();

                imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
                
                key = 1;
                systemInitialized = true;
                ROS_INFO("systemInitialized");
                return;
            }
            gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), gpsPose, gcorrectionNoise);
            graphFactors.add(pose_factor);
            ROS_INFO("update gpspose");
        }

        // add velocity
        // cout << "odomVel_:\n" << odomVel_ << endl;
        gtsam::PriorFactor<gtsam::Vector3> priorVel(V(key), odomVel_, priorVelNoise);
        graphFactors.add(priorVel);
        

        //将积分器imu变量添加到优化器中
        const gtsam::PreintegratedImuMeasurements& preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imuIntegratorOpt_);
        gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu);
        graphFactors.add(imu_factor);
        
        // add imu bias between factor
        graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
                         gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));
        // insert predicted values
        gtsam::NavState propState_ = imuIntegratorOpt_->predict(prevState_, prevBias_);
        prevStateOdom = gtsam::NavState(propState_.pose(), propState_.v());
        prevBiasOdom = prevBias_;
        graphValues.insert(X(key), propState_.pose());
        graphValues.insert(V(key), propState_.v());
        graphValues.insert(B(key), prevBias_);

        // cout << "pose:\n" << propState_.pose() << "vel:\n"<< propState_.v() << "bias:\n" << prevBias_ << endl;
        // cout << "\n preint_imu:\n" << preint_imu << "\n bias: \n" << gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias) << endl;
        // optimize
        optimizer.update(graphFactors, graphValues);
        optimizer.update();
        graphFactors.resize(0);
        graphValues.clear();
        // Overwrite the beginning of the preintegration for the next step.
        //更新状态信息与重置imu积分器
        gtsam::Values result = optimizer.calculateEstimate();
        prevPose_  = result.at<gtsam::Pose3>(X(key));
        prevVel_   = result.at<gtsam::Vector3>(V(key));
        prevState_ = gtsam::NavState(prevPose_, prevVel_);
        prevBias_  = result.at<gtsam::imuBias::ConstantBias>(B(key));
        // Reset the optimization preintegration object.
        imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
        // check optimization
        if (failureDetection(prevVel_, prevBias_))
        {
            resetParams();
            ROS_INFO("failureDetectied");
            return;
        }
        

        updated = true
        doneFirstOpt = true;
        key++;
        cout<<"key:"<< key << endl; 
    }

    //回调函数，将数据填入队列
    void poseHandler(const geometry_msgs::PoseStamped::ConstPtr& poseMsg)
    {
        //std::lock_guard<std::mutex> lock(mtx);
        //坐标系转换  调用utility.h中函数
        geometry_msgs::PoseStamped thisPose = poseConverter(*poseMsg);
        //转换后pose信息存入两个队列 后续使用
        if(thisPose.header.frame_id == "gnss")
            rtkPoseQue.push_back(thisPose);
        else if (thisPose.header.frame_id == "/map")
        {
            //激光位置丢失检测，如果已丢失，则不输入系统，维持系统短暂稳定性
            if(enablePub == true && systemInitialized == true && (GetDifferenceTwoPose(pose_slam.pose.pose, thisPose.pose) > initialPubGate))
            {
                cout << "position might be lost repub initial pose" << endl;
                pubInitial.publish(pose_slam);
                selfpub = true;
            }
            else
                ndtPoseQue.push_back(thisPose);
        }

    }

    
    void pointHandler(const geometry_msgs::PointStamped::ConstPtr& pointMsg)
    {
        //std::lock_guard<std::mutex> lock(mtx);
        //坐标系转换  调用utility.h中函数
        geometry_msgs::PoseStamped thisPose = pointConverter(*pointMsg);
        //转换后pose信息存入两个队列 后续使用
        pointQue.push_back(thisPose);

    }


    bool failureDetection(const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur)
    {
        Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
        if (vel.norm() > 30)
        {
            ROS_WARN("Large velocity, reset IMU-preintegration!");
            return true;
        }

        Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(), biasCur.accelerometer().z());
        Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(), biasCur.gyroscope().z());
        if (ba.norm() > 1.0 || bg.norm() > 1.0)
        {
            ROS_WARN("Large bias, reset IMU-preintegration!");
            return true;
        }

        return false;
    }


    //imu回调函数，输入信息存入imu队列，计算imu积分里程计；发布，存入队列
    void imuHandler(const sensor_msgs::Imu::ConstPtr& imu_raw)
    {
        std::lock_guard<std::mutex> lock(mtx);
        //坐标系转换  调用utility.h中函数
        sensor_msgs::Imu thisImu = imuConverter(*imu_raw);
        //转换后imu信息存入两个队列 后续使用
        imuQueOpt.push_back(thisImu);

        // predict odometry
        //根据预积分预测值
        gtsam::NavState currentState = imuIntegratorOpt_->predict(prevStateOdom, prevBiasOdom);

        // publish odometry
        //发布imu里程计 /odom_incremental
        nav_msgs::Odometry odometry;
        odometry.header.stamp = thisImu.header.stamp;
        odometry.header.frame_id = odometryFrame;
        odometry.child_frame_id = "odom_imu";

        // transform imu pose to ldiar
        //预测值currentState获得imu位姿 再由imu到雷达变换 获得雷达位姿
        //本程序文件开头定义了imu2Lidar  与params.yaml中外参矩阵有关
        gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());
        gtsam::Pose3 lidarPose = imuPose.compose(imu2Lidar);

        //IMU里程计的相关数据填充 
        odometry.pose.pose.position.x = lidarPose.translation().x();
        odometry.pose.pose.position.y = lidarPose.translation().y();
        odometry.pose.pose.position.z = lidarPose.translation().z();
        odometry.pose.pose.orientation.x = lidarPose.rotation().toQuaternion().x();
        odometry.pose.pose.orientation.y = lidarPose.rotation().toQuaternion().y();
        odometry.pose.pose.orientation.z = lidarPose.rotation().toQuaternion().z();
        odometry.pose.pose.orientation.w = lidarPose.rotation().toQuaternion().w();
        
        odometry.twist.twist.linear.x = currentState.velocity().x();
        odometry.twist.twist.linear.y = currentState.velocity().y();
        odometry.twist.twist.linear.z = currentState.velocity().z();
        odometry.twist.twist.angular.x = thisImu.angular_velocity.x + prevBiasOdom.gyroscope().x();
        odometry.twist.twist.angular.y = thisImu.angular_velocity.y + prevBiasOdom.gyroscope().y();
        odometry.twist.twist.angular.z = thisImu.angular_velocity.z + prevBiasOdom.gyroscope().z();
        imuOdomQueue.push_back(odometry);
        pubImuOdometry.publish(odometry);
    }

    //从里程计中提取线速度信息
    void odomHandler(const nav_msgs::Odometry::ConstPtr& odometry)
    {
        // imuOdomQueue.push_back(*odometry);
        gtsam::NavState propState_ = imuIntegratorOpt_->predict(prevState_, prevBias_);
        //prevVel_ = gtsam::Vector3(odometry->twist.twist.linear.x, odometry->twist.twist.linear.y, odometry->twist.twist.linear.z);
        odomVel_ = propState_.R() * gtsam::Vector3(odometry->twist.twist.linear.x, odometry->twist.twist.linear.y, odometry->twist.twist.linear.z);

        // prevStateOdom = gtsam::NavState(prevStateOdom.pose(),prevVel_);
    }
    
    void initialHandler(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& posewc)
    {

        if(selfpub == true)
            selfpub = false;
        else
        {
            enablePub = false;
            resetOptimization();
        }
    }

};





int main(int argc, char** argv)
{
    ros::init(argc, argv, "fusion");
    ROS_INFO("ros inited");
    
    Fusion ImuP;
    ROS_INFO("fusion inited");


    ROS_INFO("\033[1;32m----> IMU Preintegration Started.\033[0m");
    
    std::thread updatethread(&Fusion::updateThread, &ImuP);

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    
    updatethread.join();
    
    return 0;
}
