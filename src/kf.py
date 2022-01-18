#!/usr/bin/env python
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

rospy.init_node('kf', anonymous=True)

pub_pose_kf = rospy.Publisher('/tello_pose_kf', Twist, queue_size=10)

dt = 1.0/15
A = np.array([[1.0, dt, dt], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], dtype = np.float32)
C_position = np.array([1.0, 0.0, 0.0], dtype = np.float32).reshape(1, 3)
C_velocity = np.array([0.0, 1.0, 1.0], dtype = np.float32).reshape(1, 3)

Q_x = 0.1 * np.identity(3)
R_position_x = 0.01
R_velocity_x = 0.1
Q_y = 0.1 * np.identity(3)
R_position_y = 0.01
R_velocity_y = 0.1
Q_z = 0.1 * np.identity(3)
R_position_z = 0.01
R_velocity_z = 0.1

P = 0.2 * np.identity(3)
X = np.zeros((3, 1), dtype = np.float32)

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

drone_x = Kalman_filter(Q_x, R_position_x, R_velocity_x, P, X)
drone_y = Kalman_filter(Q_y, R_position_y, R_velocity_y, P, X)
drone_z = Kalman_filter(Q_z, R_position_z, R_velocity_z, P, X)

def get_imu_message(imu_msg):
    global drone_x, drone_y, drone_z
    drone_x.correction()
    drone_y.correction()
    drone_z.correction()
    drone_x.update_v(imu_msg.angular_velocity.x)
    drone_y.update_v(imu_msg.angular_velocity.y)
    drone_z.update_v(imu_msg.angular_velocity.z)

def get_marker_message(marker_msg):
    global drone_x, drone_y, drone_z
    drone_x.correction()
    drone_y.correction()
    drone_z.correction()
    drone_x.update_p(marker_msg.linear.x)
    drone_y.update_p(marker_msg.linear.y)
    drone_z.update_p(marker_msg.linear.z)

rospy.Subscriber("/repub_imu", ImuMsg, callback=get_imu_message, queue_size=10)
rospy.Subscriber("/tello_pose_marker", Twist, callback=get_marker_message, queue_size=10)

while not rospy.is_shutdown():
    rate = rospy.Rate(15)
    drone_x.correction()
    drone_y.correction()
    drone_z.correction()
    drone_x.predict()
    drone_y.predict()
    drone_z.predict()
    tello_pose_kf = Twist()
    tello_pose_kf.linear.x = drone_x.X[0, 0]
    tello_pose_kf.linear.y = drone_y.X[0, 0]
    tello_pose_kf.linear.z = drone_z.X[0, 0]
    pub_pose_kf.publish(tello_pose_kf)
    rate.sleep()