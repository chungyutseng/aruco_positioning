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

rospy.init_node('kf', anonymous=True)

###########################################################
dt = 1.0/15
A = np.array([[1.0, dt, dt], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], dtype = np.float32)
C_position = np.array([1.0, 0.0, 0.0], dtype = np.float32)
C_velocity = np.array([0.0, 1.0, 1.0], dtype = np.float32)
Q_position = 0.1 * np.identity(3)
Q_velocity = np.identity(3)
R_position = 0.01 * np.identity(3)
R_velocity = 0.1 * np.identity(3)

P = 1.0 * np.identity(3)
X = np.zeros((3, 1), dtype = np.float32)
X_ = np.zeros((3, 1), dtype = np.float32)
###########################################################

class Kalman_Filter:

def get_imu_message(data):
    x_velocity_imu = data.angular_velocity.x
    
    ###########################################################
    global X_, X
    ###########################################################
    
    y_velocity_imu = data.angular_velocity.y
    z_velocity_imu = data.angular_velocity.z
    

def get_marker_message(data):
    x_position = data.linear.x

    ###########################################################
    global X_, X
    ###########################################################

    y_position = data.linear.y
    z_position = data.linear.z

rospy.Subscriber("/tello_pose_imu", ImuMsg, callback=get_imu_message, queue_size=10)
rospy.Subscriber("/tello_pose_marker", Twist, callback=get_marker_message, queue_size=10)

while not rospy.is_shutdown():
    global X_
    rate = rospy.Rate(15)
    X_ = A * X
    rate.sleep()