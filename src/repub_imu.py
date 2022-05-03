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

rospy.init_node('imu', anonymous=True)

rate = rospy.Rate(15)

pub_pose_imu = rospy.Publisher('repub_imu', numpy_msg(Floats), queue_size=10)

tello_pose_imu = np.zeros((6,), dtype=np.float32)

def get_imu_message(imu_msg):
    global tello_pose_imu

    tello_pose_imu[0] = imu_msg.angular_velocity.x
    tello_pose_imu[1] = imu_msg.angular_velocity.y
    tello_pose_imu[2] = imu_msg.angular_velocity.z

    quaternion = (imu_msg.orientation.w, imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]

    roll = math.degrees(roll)
    pitch = math.degrees(pitch)
    yaw = math.degrees(yaw)

    if roll > 0:
        roll = roll - 180
    elif roll < 0:
        roll = roll + 180
    roll = -roll

    ##############################################
            # roll is the one to be used #
    ##############################################

    # roll, pitch, yaw in degrees
    tello_pose_imu[3] = roll
    tello_pose_imu[4] = pitch
    tello_pose_imu[5] = yaw

rospy.Subscriber("tello/imu", ImuMsg, callback=get_imu_message, queue_size=10)

while not rospy.is_shutdown():
    pub_pose_imu.publish(tello_pose_imu)
    rate.sleep()