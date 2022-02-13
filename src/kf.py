#!/usr/bin/env python3
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import cv2
import cv2.aruco as aruco
import math
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import time
from std_msgs.msg import Empty as EmptyMsg
from sensor_msgs.msg import Imu as ImuMsg
import tf
import math
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

rospy.init_node('kf', anonymous=True)

rate = rospy.Rate(15)

###########################
# past_t = time.time()
###########################

# pub_pose_kf = rospy.Publisher('/tello_pose_kf', Twist, queue_size=10)

# x, y, z, and yaw
# x, y, z in meter; yaw in degrees
tello_pose_kf = np.zeros((13,), dtype=np.float32)
temp_imu = np.zeros((6,), dtype=np.float32)

pub_pose_kf = rospy.Publisher('/tello_pose_kf', numpy_msg(Floats), queue_size=10)

dt = 1.0/15


#####################################
         # for x, y, and z #
#####################################
# A = np.array([[1.0, dt, dt], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], dtype = np.float32)
A = np.array([[1.0, dt, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], dtype = np.float32)
C_position = np.array([1.0, 0.0, 0.0], dtype = np.float32).reshape(1, 3)
C_velocity = np.array([0.0, 1.0, 1.0], dtype = np.float32).reshape(1, 3)

# Q_x = 0.1 * np.identity(3)
Q_x_pos = 0.01
Q_x_vel = 0.01
Q_x_vel_drift = 0.0001
Q_x = np.array([[Q_x_pos, 0.0, 0.0], [0.0, Q_x_vel, 0.0], [0.0, 0.0, Q_x_vel_drift]], dtype = np.float32)

R_position_x = 1.5
R_velocity_x = 1

# Q_y = 0.1 * np.identity(3)
Q_y_pos = 0.01
Q_y_vel = 0.01
Q_y_vel_drift = 0.0001
Q_y = np.array([[Q_y_pos, 0.0, 0.0], [0.0, Q_y_vel, 0.0], [0.0, 0.0, Q_y_vel_drift]], dtype = np.float32)

R_position_y = 1
R_velocity_y = 1

# Q_z = 0.1 * np.identity(3)
Q_z_pos = 0.001
Q_z_vel = 0.001
Q_z_vel_drift = 0.0000001
Q_z = np.array([[Q_z_pos, 0.0, 0.0], [0.0, Q_z_vel, 0.0], [0.0, 0.0, Q_z_vel_drift]], dtype = np.float32)

R_position_z = 1.5
R_velocity_z = 1

P = 0.2 * np.identity(3)
X = np.zeros((3, 1), dtype = np.float32)

#####################################
             # for yaw #
#####################################
A_yaw = np.array([[1.0, dt], [0.0, 1.0]], dtype = np.float32)
C_yaw = np.array([1.0, 0.0], dtype = np.float32).reshape(1, 2)
Q_yaw = 0.1 * np.identity(2)
R_yaw_marker = 2.5
R_yaw_imu = 1

P_yaw = 0.2 * np.identity(2)
X_yaw = np.zeros((2, 1), dtype = np.float32)

class Kalman_filter:

    def __init__(self, Q, R_position, R_velocity, P, X):
        self.Q = Q
        self.R_position = R_position
        self.R_velocity = R_velocity
        self.P = P
        self.X = X

    def update_v(self, measure_v):
        temp = self.P.dot(C_velocity.T)
        K = temp / (C_velocity.dot(self.P.dot(C_velocity.T)) + self.R_velocity)
        self.X = self.X + K * (measure_v - C_velocity.dot(self.X))
        I = np.identity(3)
        temp = I - K.dot(C_velocity)
        self.P = temp.dot(self.P)

    def update_p(self, measure_p):
        # now_t = time.time()
        # if ((now_t - past_t) < 20 or (now_t - past_t) > 25):
        temp = self.P.dot(C_position.T)
        K = temp / (C_position.dot(self.P.dot(C_position.T)) + self.R_position)
        self.X = self.X + K * (measure_p - C_position.dot(self.X))
        I = np.identity(3)
        temp = I - K.dot(C_position)
        self.P = temp.dot(self.P)

    def predict(self):
        self.X = A.dot(self.X)
        self.P = A.dot(self.P.dot(A.T)) + self.Q

    def correction(self):
        self.P[0, 1] = 0
        self.P[0, 2] = 0
        self.P[1, 0] = 0
        self.P[1, 2] = 0
        self.P[2, 0] = 0
        self.P[2, 1] = 0

class Kalman_filter_yaw:

    def __init__(self, Q_yaw, R_yaw_marker, R_yaw_imu, P_yaw, X_yaw):
        self.Q_yaw = Q_yaw
        self.R_yaw_marker = R_yaw_marker
        self.R_yaw_imu = R_yaw_imu
        self.P_yaw = P_yaw
        self.X_yaw = X_yaw

    def update_yaw_imu(self, measure_yaw_imu):
        temp = self.P_yaw.dot(C_yaw.T)
        K = temp / (C_yaw.dot(self.P_yaw.dot(C_yaw.T)) + self.R_yaw_imu)
        self.X_yaw = self.X_yaw + K * (measure_yaw_imu - C_yaw.dot(self.X_yaw))
        I = np.identity(2)
        temp = I - K.dot(C_yaw)
        self.P_yaw = temp.dot(self.P_yaw)

    def update_yaw_marker(self, measure_yaw_marker):
        temp = self.P_yaw.dot(C_yaw.T)
        K = temp / (C_yaw.dot(self.P_yaw.dot(C_yaw.T)) + self.R_yaw_marker)
        self.X_yaw = self.X_yaw + K * (measure_yaw_marker - C_yaw.dot(self.X_yaw))
        I = np.identity(2)
        temp = I - K.dot(C_yaw)
        self.P_yaw = temp.dot(self.P_yaw)

    def predict_yaw(self):
        self.X_yaw = A_yaw.dot(self.X_yaw)
        self.P_yaw = A_yaw.dot(self.P_yaw.dot(A_yaw.T)) + self.Q_yaw

    def correction_yaw(self):
        self.P_yaw[0, 1] = 0
        self.P_yaw[1, 0] = 0

drone_x = Kalman_filter(Q_x, R_position_x, R_velocity_x, P, X)
drone_y = Kalman_filter(Q_y, R_position_y, R_velocity_y, P, X)
drone_z = Kalman_filter(Q_z, R_position_z, R_velocity_z, P, X)
drone_yaw = Kalman_filter_yaw(Q_yaw, R_yaw_marker, R_yaw_imu, P_yaw, X_yaw)

def get_imu_message(imu_msg):
    global drone_x, drone_y, drone_z, drone_yaw
    global temp_imu
    temp_imu = imu_msg.data
    drone_x.correction()
    drone_y.correction()
    drone_z.correction()
    drone_yaw.correction_yaw()
    # drone_x.update_v(imu_msg.angular_velocity.x)
    # drone_y.update_v(imu_msg.angular_velocity.y)
    # drone_z.update_v(imu_msg.angular_velocity.z)
    # drone_x.update_v(imu_msg.linear.x)
    # drone_y.update_v(imu_msg.linear.y)
    # drone_z.update_v(imu_msg.linear.z)
    # drone_yaw.update_yaw_imu(imu_msg.angular.x)
    drone_x.update_v(temp_imu[0])
    drone_y.update_v(temp_imu[1])
    drone_z.update_v(temp_imu[2])
    drone_yaw.update_yaw_imu(temp_imu[3])

def get_marker_message(marker_msg):
    global drone_x, drone_y, drone_z, drone_yaw
    temp = marker_msg.data
    drone_x.correction()
    drone_y.correction()
    drone_z.correction()
    drone_yaw.correction_yaw()
    drone_x.update_p(temp[0])
    drone_y.update_p(temp[1])
    drone_z.update_p(temp[2])
    drone_yaw.update_yaw_marker(temp[4])
    # drone_x.update_p(marker_msg.linear.x)
    # drone_y.update_p(marker_msg.linear.y)
    # drone_z.update_p(marker_msg.linear.z)
    # drone_yaw.update_yaw_marker(marker_msg.angular.y)

# rospy.Subscriber("/repub_imu", ImuMsg, callback=get_imu_message, queue_size=10)
# rospy.Subscriber("/repub_imu", Twist, callback=get_imu_message, queue_size=10)
# rospy.Subscriber("/tello_pose_marker", Twist, callback=get_marker_message, queue_size=10)
rospy.Subscriber("/repub_imu", numpy_msg(Floats), callback=get_imu_message)
rospy.Subscriber("/tello_pose_marker", numpy_msg(Floats), callback=get_marker_message)

while not rospy.is_shutdown():
    drone_x.correction()
    drone_y.correction()
    drone_z.correction()
    drone_yaw.correction_yaw()
    drone_x.predict()
    drone_y.predict()
    drone_z.predict()
    drone_yaw.predict_yaw()
    # tello_pose_kf = Twist()
    # tello_pose_kf.linear.x = drone_x.X[0, 0]
    # tello_pose_kf.linear.y = drone_y.X[0, 0]
    # tello_pose_kf.linear.z = drone_z.X[0, 0]
    # tello_pose_kf.angular.z = drone_yaw.X_yaw[0, 0]
    tello_pose_kf[0] = drone_x.X[0, 0]
    tello_pose_kf[1] = drone_y.X[0, 0]
    tello_pose_kf[2] = drone_z.X[0, 0]
    tello_pose_kf[3] = drone_yaw.X_yaw[0, 0]
    tello_pose_kf[4] = drone_x.X[1, 0] # x velocity
    tello_pose_kf[5] = drone_x.X[2, 0] # x velocity drift
    tello_pose_kf[6] = drone_y.X[1, 0] # y velocity
    tello_pose_kf[7] = drone_y.X[2, 0] # y velocity drift
    tello_pose_kf[8] = drone_z.X[1, 0] # z velocity
    tello_pose_kf[9] = drone_z.X[2, 0] # z velocity drift
    tello_pose_kf[10] = temp_imu[0]
    tello_pose_kf[11] = temp_imu[1]
    tello_pose_kf[12] = temp_imu[2]
    pub_pose_kf.publish(tello_pose_kf)
    rate.sleep()