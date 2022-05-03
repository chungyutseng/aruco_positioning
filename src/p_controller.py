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
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from tello_driver.msg import TelloStatus
import functools

rospy.init_node('p_controller', anonymous=True)

rate = rospy.Rate(15)

pub_vel = rospy.Publisher("tello/cmd_vel", Twist, queue_size=10)
pub_height = rospy.Publisher("height", Float32, queue_size=10)

height = 0.0
desired_height = 1.25

kp = 2.1

vel_max_linear = 2.3
vel_min_linear = 0.25

vel_msg = Twist()
pc_on = 0.0

def get_pc_on(data):
    global pc_on
    pc_on = data.data

def get_height(msg):
    global height
    height = msg.height_m

def p_controller(c_height, d_height, kp_gain):
    global vel_msg
    vel_msg.linear.z = kp_gain * (d_height - c_height)
    sign = functools.partial(math.copysign, 1)
    if abs(vel_msg.linear.z) > vel_max_linear:
        vel_msg.linear.z = sign(vel_msg.linear.z) * vel_max_linear
    if abs(vel_msg.linear.z) < vel_min_linear:
        vel_msg.linear.z = sign(vel_msg.linear.z) * vel_min_linear
    if abs(d_height - c_height) < 0.1:
        vel_msg.linear.z = 0.0

rospy.Subscriber("tello/status", TelloStatus, callback=get_height)
rospy.Subscriber('pc_on', Float32, callback=get_pc_on)

while not rospy.is_shutdown():
    p_controller(height, desired_height, kp)
    if (pc_on == 1.0):
        pub_vel.publish(vel_msg)
    pub_height.publish(height)
    rate.sleep()