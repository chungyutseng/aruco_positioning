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

rospy.init_node('imu', anonymous=True)

pub_pose_imu = rospy.Publisher('/tello_pose_imu', ImuMsg, queue_size=10)

tello_pose_imu = ImuMsg()

def get_imu_message(imu_msg):
    global tello_pose_imu
    tello_pose_imu.angular_velocity.x = imu_msg.angular_velocity.x
    tello_pose_imu.angular_velocity.y = imu_msg.angular_velocity.y
    tello_pose_imu.angular_velocity.z = imu_msg.angular_velocity.z
    quaternion = (imu_msg.orientation.w, imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]

    # use linear_acceleration as roll, pitch, and yaw
    tello_pose_imu.linear_acceleration.x = roll
    tello_pose_imu.linear_acceleration.y = pitch
    tello_pose_imu.linear_acceleration.z = yaw

rospy.Subscriber("/tello/imu", ImuMsg, callback=get_imu_message, queue_size=10)

while not rospy.is_shutdown():
    rate = rospy.Rate(15)
    pub_pose_imu.publish(tello_pose_imu)
    rate.sleep()