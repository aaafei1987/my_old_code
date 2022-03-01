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
        
def normalize_angle(angle):
    res = angle
    while res > math.pi:
        res -= 2.0*math.pi
    while res < -math.pi:
        res += 2.0*math.pi
    return res

def normalize_array(input_arr):
    output_arr = np.zeros(len(input_arr)).reshape(3,1)
    for i in range(len(input_arr)):
        output_arr[i] = normalize_angle(input_arr[i])
    return output_arr

class ekf_imu():
    def __init__(self):
        rospy.init_node('EKF_imu', log_level=rospy.DEBUG)

        # Cleanup when termniating the node
        rospy.on_shutdown(self.shutdown)

        # variable
        self.mutex_imu_noise = 0.0
        self.rot_output = np.array([0.0,0.0,0.0]).reshape(3,1)       #combined output([roll,pitch,yaw]) 
        self.vel_angle = np.array([0.0,0.0,0.0]).reshape(3,1)        #control input  ([vela_x,vela_y,vela_z]) from imu
        self.rot_ob = np.array([0.0,0.0,0.0]).reshape(3,1)           #observe input  ([roll,pitch,yaw]) from ndt
        self.rot_cov = np.diag([0.001,0.001,0.001])
        self.update_stamp = rospy.Time.now()
        self.first_pose = True
        self.pose_counter = 0

        # param
        self.freq = rospy.get_param("imu_freq",50)
        self.pose_topic = rospy.get_param("~pose_topic","gnss_pose")
        self.use_imu_setup = rospy.get_param("use_imu_setup",True)
        self.imu_noise = (rospy.get_param("imu_noise",0.01)/self.freq)**2   #每次接收imu数据引入的方差（0.03°)**2 = >  (0.00051)**2

        # subscriber
        rospy.Subscriber("imu_raw1", Imu, self.imuCallback)
        rospy.Subscriber(self.pose_topic, PoseStamped, self.poseCallback)
        rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.initialCallback)

        # Publisher
        self.imu_pub = rospy.Publisher('imu_raw',Imu,queue_size=5)


        while not rospy.is_shutdown():
            rospy.spin()

    def imuCallback(self,msg):
        imu_now = msg
        self.vel_angle = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]).reshape(3,1)

        self.rot_output = self.motion_model(self.rot_output, self.vel_angle, msg.header.stamp)
        self.mutex_imu_noise += self.imu_noise 
        self.update_stamp = msg.header.stamp
        rot_quat = angle_to_quat(self.rot_output)
        imu_now.orientation.x = rot_quat[0]
        imu_now.orientation.y = rot_quat[1]
        imu_now.orientation.z = rot_quat[2]
        imu_now.orientation.w = rot_quat[3]
        imu_now.orientation_covariance = tuple(self.rot_cov.flatten())
        imu_now.header.stamp = rospy.Time.now()

        if (self.use_imu_setup == False or self.pose_counter > 30):
            self.imu_pub.publish(imu_now)


    def poseCallback(self,msg):
        if(self.first_pose == True):
            self.first_pose = False
            self.rot_base = np.array(quat_to_angle(msg.pose.orientation)).reshape(3,1)
        delta = normalize_array(np.array(quat_to_angle(msg.pose.orientation)).reshape(3,1) - self.rot_ob)*90
        self.cov_dynamic = np.diag([pow(delta[0],2)+0.001,pow(delta[1],2)+0.001,pow(delta[2],2)+0.001])
        self.rot_ob = normalize_array(np.array(quat_to_angle(msg.pose.orientation)).reshape(3,1) - self.rot_base)

        self.rot_output = normalize_array(self.motion_model(self.rot_output, self.vel_angle, msg.header.stamp))
        self.rot_output, self.rot_cov = self.ekf_estimation(self.rot_output, self.rot_cov, self.rot_ob)
        self.update_stamp = msg.header.stamp
        self.pose_counter += 1
        
    def initialCallback(self,msg):
        if (self.pose_topic == "ndt_pose"):
            self.first_pose == True
            self.rot_output = np.array([0.0,0.0,0.0]).reshape(3,1)   
            self.pose_counter == 0
        
    def shutdown(self):
        # Stop the robot
        try:
            rospy.loginfo("Stopping the EKF node...")
            rospy.sleep(2)
        except:
            pass
        rospy.loginfo("Shutting down Stm32 Node...")


    def motion_model(self,x,u,t):
        dt = (t - self.update_stamp).to_sec()

        B = np.matrix([[dt, 0.0, 0.0],

                    [0.0, dt, 0.0],

                    [0.0, 0.0, dt]])

        x = x + B*u

        return x

    def observe_model(self,x,z):

        H = np.matrix([[1.0, 0.0, 0.0],

                        [0.0, 1.0, 0.0],

                        [0.0, 0.0, 1.0]])


    def ekf_estimation(self,meanEst,covEst,z):

        # predict



        Q = np.diag([self.mutex_imu_noise, self.mutex_imu_noise, self.mutex_imu_noise])

        self.mutex_imu_noise = 0.0

        pPre = covEst + Q      

        s = pPre +self.cov_dynamic

        k = pPre * np.linalg.inv(s)



        zPre = meanEst

        meanEst = normalize_array(meanEst + k * normalize_array((z - zPre))

        covEst = (np.eye(len(meanEst)) - k) * pPre

        return meanEst,covEst
        # # predict

        # xPre=motion_model(meanEst,u)

        # jMo=JacoMo(meanEst,u)   # Jacobin of motion model

        # Q = np.diag([pow(self.mutex_imu_noise,2),pow(self.mutex_imu_noise,2),pow(self.mutex_imu_noise,2)])

        # self.mutex_imu_noise = 0.0

        # pPre=jMo*covEst*jMo.T + Q      

        # jOb=JacoOb(meanEst)

        # s=jOb*pPre*jOb.T+self.cov_dynamic

        # k=pPre*jOb.T*np.linalg.inv(s)

        # # update estimate

        # H=np.matrix([

        #         [1,0,0],

        #         [0,1,0],

        #         [0,0,1]

        #         ])

        # zPre=H*xPre

        # meanEst=xPre+k*(z-zPre)

        # covEst=(np.eye(len(meanEst)) - k*jOb)*pPre

        # return meanEst,covEst




# def JacoMo(x, u):

#     """

#     Jacobian of Motion Model



#     motion model

#     roll_{t+1} = roll_t+vel_r*dt

#     pitch_{t+1} = pitch_t+vel_p*dt

#     yaw_{t+1} = yaw_t+vel_y*dt



#     """

#     jF = np.matrix([[1.0, 0.0, 0.0],

#         [0.0, 1.0, 0.0],

#         [0.0, 0.0, 1.0]])



#     return jF





# def JacoOb(x):

#     # Jacobian of Observation Model

#     jH = np.matrix([

#         [1, 0, 0],

#         [0, 1, 0],

#         [0, 0, 1],

#     ])



#     return jH


if __name__=="__main__":
    ekf_node = ekf_imu()
