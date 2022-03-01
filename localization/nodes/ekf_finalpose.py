#! /usr/bin/env python2.7
# encoding=utf-8



import numpy as np
import PyKDL

import math

import rospy
from sensor_msgs.msg import Imu
from custom_msgs.msg import robotic_control_msgs
from geometry_msgs.msg import Quaternion, PoseWithCovarianceStamped, PoseStamped, Pose, Twist, TwistStamped
from std_msgs.msg import Header



def point_to_array(point):

    """
    将geometry_msgs/Point类型数据转换为numpy/Array
 
    :param point: 输入位姿
    :type : geometry_msgs/Point

    :return: 列向量(x,y,z,r,p,y)^T
    :rtype: numpy/Array

    """

    return np.array((point.x, point.y, point.z)
        

def pose_to_array(pose):

    """
    将geometry_msgs/Pose类型数据转换为numpy/Array
 
    :param pose: 输入位姿
    :type : geometry_msgs/Pose

    :return: 列向量(x,y,z,r,p,y)^T
    :rtype: numpy/Array

    """

    rot = PyKDL.Rotation.Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    return np.array((pose.position.x,pose.position.y,pose.position.z)+rot.GetRPY()).reshape(6,1)
        
def array_to_pose(arr):

    """
    将 numpy/Array 转换为 geometry_msgs/Pose 类型数据
 
    :param arr: 输入列向量(x,y,z,r,p,y)^T
    :type : numpy/Array

    :return: 输出位姿
    :rtype: geometry_msgs/Pose 
 
    """

    pose = Pose()
    rot = (PyKDL.Rotation.RPY(arr[3], arr[4], arr[5])).GetQuaternion()
    pose.orientation.x = rot[0]
    pose.orientation.y = rot[1]
    pose.orientation.z = rot[2]
    pose.orientation.w = rot[3]
    pose.position.x = arr[0].tolist()[0][0]
    pose.position.y = arr[1].tolist()[0][0]
    pose.position.z = arr[2].tolist()[0][0]
    return pose

def array_sub(arr1,arr2):

    """
    将 numpy/Array 格式的位姿取标准化差值
 
    :param arr1,arr2: 输入列向量(x,y,z,r,p,y)^T
    :type : numpy/Array

    :return: 输出位姿向量(x,y,z,r,p,y)^T
    :rtype: numpy/Array
 
    """

    out = arr1 - arr2
    out[3:6] = normalize_array(out[3:6])
    return out

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

    """
    将角度标准化
 
    :param angle,: 输入角度（rad）
    :type : float

    :return: 输出角度（rad）
    :rtype: float
 
    """

    res = angle
    while res > math.pi:
        res -= 2.0*math.pi
    while res < -math.pi:
        res += 2.0*math.pi
    return res

def normalize_array(input_arr):

    """
    将 numpy/Array 格式的位姿进行角度标准化
 
    :param angle,: 输入列向量(x,y,z,r,p,y)^T
    :type : numpy/Array

    :return: 输出位姿向量(x,y,z,r,p,y)^T
    :rtype: numpy/Array
 
    """
    
    output_arr = np.zeros(len(input_arr)).reshape(3,1)
    for i in range(len(input_arr)):
        output_arr[i] = normalize_angle(input_arr[i])
    return output_arr

class ekf_finalpose():
    def __init__(self):
        rospy.init_node('EKF_finalpose', log_level=rospy.DEBUG)

        # Cleanup when termniating the node
        rospy.on_shutdown(self.shutdown)

        # param
        self.use_rtk = rospy.get_param("~use_rtk", True)
        self.rot_tolerance = rospy.get_param("~rot_tolerance",0.18)
        self.gnss_tunnel_length = rospy.get_param("~gnss_tunnel_length",10)
        self.gnss_tran_noise = pow(rospy.get_param("~gnss_tran_noise", 1.0), 2)
        self.gnss_rot_noise = pow(rospy.get_param("~gnss_rot_noise", 0.01), 2)
        self.gnss_cov = np.diag([self.gnss_tran_noise, self.gnss_tran_noise, self.gnss_tran_noise, 10000.0, 10000.0, self.gnss_rot_noise])
        self.point_cov = np.diag([self.gnss_tran_noise, self.gnss_tran_noise, self.gnss_tran_noise, 10000.0, 10000.0, 10000,0])

        # variable
        self.first_ndt = True
        self.first_gnss = True

        self.pose_change_counter = 0
        self.score = 0.0
        self.gnss_counter = 0 

        self.update_stamp = rospy.Time.now()
        self.gnsstimer = rospy.Time.now()
        self.ndttimer = rospy.Time.now()
        self.ndt_enable_timmer = rospy.Time.now()

        self.vel = np.array([0.0,0.0]).reshape(2,1)                             #control input  ([vx,wya])

        self.pose_ob = np.array([0.0,0.0,0.0,0.0,0.0,0.0]).reshape(6,1)         #observe input  ([x,y,z,roll,pitch,yaw])
        self.pose_output = np.array([0.0,0.0,0.0,0.0,0.0,0.0]).reshape(6,1)     #combined output([x,y,z,roll,pitch,yaw])
        self.pose_ndt = np.array([0.0,0.0,0.0,0.0,0.0,0.0]).reshape(6,1)
        self.pose_gnss = np.array([0.0,0.0,0.0,0.0,0.0,0.0]).reshape(6,1)
        self.pose_last = np.array([0.0,0.0,0.0,0.0,0.0,0.0]).reshape(6,1)
        self.gnss_list = [np.array([0.0,0.0,0.0,0.0,0.0,0.0]).reshape(6,1)] * 10

        self.pose_cov = np.diag([0.001,0.001,0.001,0.001,0.001,0.001])
        self.cov_ob = np.diag([0.001,0.001,0.001,0.001,0.001,0.001])


        # subscriber
        rospy.Subscriber("ndt_pose", PoseStamped, self.ndtCallback)
        rospy.Subscriber("control_status", TwistStamped, self.velCallback)
        if(self.use_rtk):
            rospy.Subscriber("gnss_pose", PoseStamped, self.gnssCallback)
        else:
            rospy.Subscriber("gps_point", PointStamped, self.pointCallback)

        # Publisher
        self.pose_pub = rospy.Publisher('pose_combined',PoseWithCovarianceStamped,queue_size=1)
        self.pekf_pub = rospy.Publisher('pose_ekf',PoseStamped,queue_size=1)
        self.vel_pub = rospy.Publisher('cmd_vel',Twist,queue_size=5)
        self.emergency_pub = rospy.Publisher('robotic_control',robotic_control_msgs,queue_size=5)
        self.initial_pub = rospy.Publisher('initialpose',PoseWithCovarianceStamped,queue_size=1)
        self.tiptop_pub = rospy.Publisher(rospy.get_name() + '_tiptop', Header, queue_size = 1)

    	self.seq = 0
        rospy.Timer(rospy.Duration(1), self.publishTiptop)

        while not rospy.is_shutdown():
            rospy.spin()

    def publishTiptop(self, event):
        msg = Header()
        msg.seq = self.seq
        msg.frame_id = 'tiptop'
        msg.stamp = rospy.Time.now()
        self.tiptop_pub.publish(msg)
        self.seq = self.seq + 1        
        

    def velCallback(self,msg):
        # vel = Twist()
        self.vel = np.array([msg.twist.linear.x, msg.twist.angular.z]).reshape(2,1)
        # vel = Twist()
        # self.vel = np.array([msg.twist.linear.x, -(msg.twist.linear.x * math.tan(msg.twist.angular.z)) / self.wheel_base]).reshape(2,1)

        # vel.linear.x = msg.twist.linear.x
        # vel.angular.z = -(msg.twist.linear.x * math.tan(msg.twist.angular.z)) / self.wheel_base
        # self.vel_pub.publish(vel)

    def ndtCallback(self,msg):
        if (self.first_ndt == True):
            self.ndttimer = msg.header.stamp
            self.pose_ndt = pose_to_array(msg.pose)
            delta = array_sub(pose_to_array(msg.pose), self.pose_ndt)
            alpha = np.array([0.0,0.0,0.0,0.0,0.0,0.0]).reshape(6,1)
        else:
            dt = (msg.header.stamp - self.ndttimer).to_sec()
            self.ndttimer = msg.header.stamp
            self.pose_ndt = self.motion_model(self.pose_ndt, self.vel, dt)
            delta = array_sub(pose_to_array(msg.pose), self.pose_ndt) 
            alpha = array_sub(self.pose_gnss, self.pose_ndt) 
            self.pose_ndt = pose_to_array(msg.pose)
        #计算测量误差
        """
        ndt协方差包含两部分：
            ndt自身的置信度产生的协方差：
                通过上一帧的ndt位姿输入与当前的速度控制输入，预测当前的位姿。使用预测位姿与实际输入的偏差作为制定协方差的噪声输入。

            根据与gnss位置差值的附加协方差，分段处理：
                1.gnss位置失效：
                    gnss超过1s没有信息返回时，只考虑ndt自身的置信度产生的噪声。
                2.gnss与ndt位置有一定偏差：
                    当定位误差在在10m以内，角度误差不超过10°（可设置的参数）时，根据相对距离产生最大0.1m的附加误差协方差，以适应地图与gnss坐标不能完全匹配的现状
                3.gnss与ndt位置偏差较大：
                    误差在10m以上，角度偏差超过10°时，将产生最小1m的附加协方差，以进行全局的重定位
    
        """
    
        if ((msg.header.stamp - self.ndt_enable_timmer).to_sec()>1) :
            pass
        elif (np.sum((np.abs(alpha).reshape(2,3))[0,0:3])<10 and (np.abs(alpha).reshape(2,3))[1,2] < self.rot_tolerance ):
            delta = delta + np.square(alpha)/100
        else:
            delta = delta + np.square(alpha)/10
        self.cov_ob = np.diag([(pow(delta[0,0],2) + 0.001), 
                            (pow(delta[1,0],2) + 0.001), 
                            (pow(delta[2,0],2) + 0.001), 
                            (pow(delta[3,0],2) + 0.001), 
                            (pow(delta[4,0],2) + 0.001), 
                            (pow(delta[5,0],2) + 0.001)])
        # print (delta)
        self.Posehandler(msg)
        self.first_ndt = False

    def gnssCallback(self,msg):
        if (self.first_gnss == True):
            self.cov_ob = self.gnss_cov
            self.gnss_list[ self.gnss_tunnel_length -1 ] = pose_to_array(msg.pose)
            self.first_gnss = False
        elif ((msg.header.stamp - self.gnsstimer).to_sec() < 1) :
            dt = (msg.header.stamp - self.gnsstimer).to_sec()
            for i in range( self.gnss_tunnel_length - 1 ):
                self.gnss_list[ i ] = self.motion_model(self.gnss_list[ i + 1 ], self.vel, dt)
            self.gnss_list[ self.gnss_tunnel_length -1 ] = pose_to_array(msg.pose)
            if (self.gnss_counter >= self.gnss_tunnel_length - 1 ):
                delta = array_sub(pose_to_array(msg.pose), self.gnss_list[ 0 ])
                print np.sum(np.abs(delta))
                self.cov_ob = self.gnss_cov + np.diag([(pow(delta[0,0],2)), 
                                (pow(delta[1,0],2)), 
                                (pow(delta[2,0],2)), 
                                (pow(delta[3,0],2)), 
                                (pow(delta[4,0],2)), 
                                (pow(delta[5,0],2))])
                self.Posehandler(msg)
                if( np.sum(np.abs(delta)) < 0.4 ):
                    self.pose_gnss = pose_to_array(msg.pose)
                    self.ndt_enable_timmer = msg.header.stamp
        else:
            self.gnss_counter = 0
            pass

        self.gnsstimer = msg.header.stamp
        if (self.gnss_counter < self.gnss_tunnel_length):
        	self.gnss_counter += 1 


    def pointCallback(self,msg):
        if (self.first_gnss == True):
            self.cov_ob = self.point_cov
            self.first_gnss = False
        else:
            self.cov_ob = self.point_cov
            posfrompoint = PoseStamped()
            posfrompoint.header = msg.header
            posfrompoint.pose.position = msg.point
            posfrompoint.pose.orientation = array_to_pose(self.pose_output).orientation
            self.Posehandler(posfrompoint)
            if( np.sum(np.abs(delta)) < 0.4 ):
                self.pose_gnss = point_to_array(posfrompoint.pose)
                self.ndt_enable_timmer = msg.header.stamp



    def Posehandler(self,msg):
        if (self.first_ndt == True and self.first_gnss == True):
            self.pose_output = pose_to_array(msg.pose)
            dt = 0
            self.pose_output = self.motion_model(self.pose_output, self.vel, dt)
            self.pose_last = self.pose_output
        else:
            dt = (msg.header.stamp - self.update_stamp).to_sec()
            if dt<0:
                dt = 0
            self.pose_output = self.motion_model(self.pose_output, self.vel, dt)
            self.pose_ob = pose_to_array(msg.pose)
            #计算估计误差
            self.cov_motion = np.diag([0.1,  0.1, 0.001, 0.001, 0.001, 0.01])**2
            self.score = np.sum(np.abs(self.pose_cov))
            # print("score:%f") %self.score
            self.pose_output, self.pose_cov = self.ekf_estimation(self.pose_output, self.pose_cov, self.pose_ob, dt)
            
        self.update_stamp = msg.header.stamp
        output_pose = PoseWithCovarianceStamped()
        output_pose.header.stamp = rospy.Time.now()
        output_pose.header.frame_id = "map"
        output_pose.pose.pose = array_to_pose(self.pose_output)
        output_pose.pose.covariance = self.pose_cov.reshape(1,36).tolist()[0]
        if(np.abs(array_sub(self.pose_last, self.pose_output))[0,0] + np.abs(array_sub(self.pose_last, self.pose_output))[1,0] >5 ):
            print("偏差：")
            print(np.abs(array_sub(self.pose_last, self.pose_output))[0,0] + np.abs(array_sub(self.pose_last, self.pose_output))[1,0])
            print("dt:")
            print(dt)
            print("predict_dif:") 
            print(self.predict_dif)
            print("actual_dif:") 
            print(array_sub(self.pose_output, self.pose_ob) )
            print(self.vel)
            print("pitch:%f;  yaw:%f") %(self.pose_output[4,0],self.pose_output[5,0])
        self.pose_last = self.pose_output
        self.pose_pub.publish(output_pose)


        poseekf = PoseStamped()
        poseekf.header.stamp = rospy.Time.now()
        poseekf.header.frame_id = "map"
        poseekf.pose = array_to_pose(self.pose_output)
        self.pekf_pub.publish(poseekf)
        beta = array_sub(self.pose_output, self.pose_ndt) 
        if (np.sum((np.abs(beta).reshape(2,3))[0,0:2])>9 or (np.abs(beta).reshape(2,3))[1,2] > self.rot_tolerance ):
            self.pose_change_counter +=1
            if (self.pose_change_counter%100 == 8):
                self.initial_pub.publish(output_pose)
            if (self.pose_change_counter%40 == 10):
                emr = robotic_control_msgs()
                emr.obj = 0
                emr.status = 1
                self.emergency_pub.publish(emr)
        else:
            self.pose_change_counter = 0


    def shutdown(self):
        # Stop the robot
        try:
            rospy.loginfo("Stopping the EKF node...")
            rospy.sleep(2)
        except:
            pass
        rospy.loginfo("Shutting down Stm32 Node...")


    def motion_model(self,x,u,dt):
        """
        x{t+1} =  x{t} + v * cos(yaw + wya * dt / 2) * cos(pitch)*dt

        y{t+1} =  y{t} + v*sin(yaw + wya * dt / 2)*cos(pitch)*dt

        z{t+1} =  z{t} + v*sin(pitch)*dt

        roll_{t+1} = roll_t

        pitch_{t+1} = pitch_t

        yaw_{t+1} = yaw_t + wya*dt
        """
        v = u[0,0]

        wya = u[1,0]

        pitch = x[4,0]

        yaw = x[5,0]

        B = np.matrix([[math.cos(yaw) * math.cos(pitch) * dt,      0.0],

                    [math.sin(yaw) * math.cos(pitch) * dt,         0.0],

                    [math.sin(pitch) * dt,                         0.0],

                    [0.0,                                          0.0],

                    [0.0,                                          0.0],

                    [0.0,                                          dt]])



        x = array_add(x, B*u)
        
        return x

    def JacoMo(self,x, u, dt):

        """

        Jacobian of Motion Model



        motion model

        x{t+1} =  x{t} + v*cos(yaw)*cos(pitch)*dt

        y{t+1} =  y{t} + v*sin(yaw)*cos(pitch)*dt

        z{t+1} =  z{t} + v*sin(pitch)*dt

        roll_{t+1} = roll_t

        pitch_{t+1} = pitch_t

        yaw_{t+1} = yaw_t + wya*dt


        so

        dx/dpitch = -v*dt*cos(yaw)*sin(pitch)

        dx/dyaw = -v*dt*sin(yaw)*cos(pitch)

        dx/dv = dt*cos(yaw)

        dx/dpitch = -v*dt*sin(yaw)*sin(pitch)

        dy/dyaw = v*dt*cos(yaw)*cos(pitch)

        dy/dv = dt*sin(yaw)


        dz/dpitch = v*dt*cos(pitch)

        dz/dyaw = 0

        dz/dv = dt*sin(pitch)

        """
        pitch = x[4, 0]

        yaw = x[5, 0]
        v = u[0, 0]
        if(yaw == 0):
            yaw = 0.01
        wya = dt * u[1, 0]/yaw

        jF = np.matrix([[1.0, 0.0, 0.0, 0.0, -v * dt * math.cos(yaw) * math.sin(pitch), -v * dt * math.sin(yaw) * math.cos(pitch)], #x

            [0.0, 1.0, 0.0, 0.0, -v * dt * math.sin(yaw) * math.sin(pitch), v * dt * math.cos(yaw) * math.cos(pitch)],              #y

            [0.0, 0.0, 1.0, 0.0, v * dt * math.cos(pitch), 0.0],                                                                    #z

            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],                                                                                         #roll
            
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],                                                                                         #pitch

            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])                                                                             #yaw


        return jF

    def ekf_estimation(self,meanEst,covEst,z,dt):


        jMo=self.JacoMo(meanEst,self.vel, dt)   # Jacobin of motion model

        pPre=jMo*covEst*jMo.T + self.cov_motion
        # update estimate

        s = pPre + self.cov_ob

        k = pPre * np.linalg.inv(s)

        zPre = meanEst

        self.predict_dif = k * array_sub(z, zPre)

        meanEst = array_add(meanEst, k * array_sub(z, zPre))

        covEst = (np.eye(len(meanEst)) - k) * pPre

        return meanEst,covEst


if __name__=="__main__":
    ekf_node = ekf_finalpose()
