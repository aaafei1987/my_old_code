#! /usr/bin/env python2.7
# encoding=utf-8


# args explanition:

#     p:covariance

#     x:state

#     z:observation

#     u:control

#     Pred:predict

#     Est:estatemation

#     Q：表示过程激励噪声的协方差，它是状态转移矩阵与实际过程之间的误差。

#     R：表示测量噪声协方差，和仪器相关的一个特性，


import numpy as np
import PyKDL

import math

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, PoseWithCovarianceStamped, PoseStamped

import matplotlib.pyplot as plt


def quat_to_angle(quat):
    rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
    return rot.GetRPY()
        
def angle_to_quat(angle):
    # print (angle)
    rot = PyKDL.Rotation.RPY(angle[0], angle[1], angle[2])
    return rot.GetQuaternion()
        
def imu_to_array(imu_data):
    rot = PyKDL.Rotation.Quaternion(imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w) 
    imu_array = np.array((imu_data.linear_acceleration.x, imu_data.linear_acceleration.y, imu_data.linear_acceleration.z) + rot.GetRPY())
    return imu_array

def array_add(arr1,arr2):

    """
    将 numpy/Array 格式的位姿取标准化求和
 
    :param arr1,arr2: 输入列向量(x,y,z,r,p,y)^T
    :type : numpy/Array

    :return: 输出位姿向量(x,y,z,r,p,y)^T
    :rtype: numpy/Array
 
    """

    out = arr1 + arr2
    out[3:6] = normalize_array(out[3:6])
    return out

def normalize_angle(angle):
    res = angle
    while res > math.pi:
        res -= 2.0*math.pi
    while res < -math.pi:
        res += 2.0*math.pi
    return res

def normalize_array(input_arr):
    output_arr = np.zeros(len(input_arr))
    for i in range(len(input_arr)):
        output_arr[i] = normalize_angle(input_arr[i])
    return output_arr

class ekf_imu():
    def __init__(self):
        rospy.init_node('EKF_imu', log_level=rospy.DEBUG)

        # Cleanup when termniating the node
        rospy.on_shutdown(self.shutdown)

        # variable
        self.imuQue = []
        self.mutex_imu_noise = 0.0
        self.rot_output = np.array([0.0,0.0,0.0]).reshape(3,1)       #combined output([roll,pitch,yaw]) 
        self.vel_angle = np.array([0.0,0.0,0.0]).reshape(3,1)        #control input  ([vela_x,vela_y,vela_z]) from imu
        self.rot_ob = np.array([0.0,0.0,0.0]).reshape(3,1)           #observe input  ([roll,pitch,yaw]) from ndt
        self.rot_cov = np.diag([0.001,0.001,0.001])
        self.update_stamp = rospy.Time.now()
        self.first_pose = True
        self.pose_counter = 0

        # param
        self.pose_topic = rospy.get_param("~pose_topic","gnss_pose")
        self.use_imu_setup = rospy.get_param("use_imu_setup",True)

        # subscriber
        rospy.Subscriber("/gx5/imu/data", Imu, self.imuCallback)
        # Publisher
        self.imu_pub = rospy.Publisher('imu_raw',Imu,queue_size=5)


        while not rospy.is_shutdown():
            rospy.spin()

    def imuCallback(self,msg):
        self.imuQue += [msg]
        if (len(self.imuQue)>1000):
            imu_sum = np.array([0, 0, 0, 0, 0, 0])
            for imudata in self.imuQue:
                imu_sum = array_add(imu_sum, imu_to_array(imudata))

            imu_mean = imu_sum/1000
            print(imu_mean)
            self.imuQue = []



    def shutdown(self):
        # Stop the robot
        try:
            rospy.loginfo("Stopping the EKF node...")
            rospy.sleep(2)
        except:
            pass
        rospy.loginfo("Shutting down Stm32 Node...")


if __name__=="__main__":
    ekf_node = ekf_imu()
