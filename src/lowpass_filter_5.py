#!/usr/bin/env python3
from itertools import count
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

rospy.init_node('lowpass_filter', anonymous=True)

rate = rospy.Rate(15)

tello_pose_marker_lp = np.zeros((6,), dtype=np.float32)
count = 0
pxs = np.zeros((4,), dtype=np.float32)
pzs = np.zeros((4,), dtype=np.float32)
pyaws = np.zeros((4,), dtype=np.float32)

pub_pose_marker_lp = rospy.Publisher("tello_pose_marker_lp", numpy_msg(Floats), queue_size=10)

def get_marker_message(marker_msg):
    global count, tello_pose_marker_lp
    global pxs, pzs, pyaws
    temp = marker_msg.data
    tello_pose_marker_lp[0] = temp[0]
    tello_pose_marker_lp[1] = temp[1]
    tello_pose_marker_lp[2] = temp[2]
    tello_pose_marker_lp[3] = temp[3]
    tello_pose_marker_lp[5] = temp[5]
    if count == 0:
        # tello_pose_marker_lp[0] = temp[0]
        # tello_pose_marker_lp[2] = temp[2]
        tello_pose_marker_lp[4] = temp[4]
        # pxs[0] = temp[0]
        # pzs[0] = temp[2]
        pyaws[0] = temp[4]
        count = count + 1
    if count == 1:
        # tello_pose_marker_lp[0] = temp[0]
        # tello_pose_marker_lp[2] = temp[2]
        tello_pose_marker_lp[4] = temp[4]
        # pxs[1] = temp[0]
        # pzs[1] = temp[2]
        pyaws[1] = temp[4]
        count = count + 1
    if count == 2:
        # tello_pose_marker_lp[0] = temp[0]
        # tello_pose_marker_lp[2] = temp[2]
        tello_pose_marker_lp[4] = temp[4]
        # pxs[2] = temp[0]
        # pzs[2] = temp[2]
        pyaws[2] = temp[4]
        count = count + 1
    if count == 3:
        # tello_pose_marker_lp[0] = temp[0]
        # tello_pose_marker_lp[2] = temp[2]
        tello_pose_marker_lp[4] = temp[4]
        # pxs[3] = temp[0]
        # pzs[3] = temp[2]
        pyaws[3] = temp[4]
        count = count + 1
    if count >= 4:
        # tello_pose_marker_lp[0] = (1.0 / 5.0) * (temp[0] + pxs[3] + pxs[2] + pxs[1] + pxs[0])
        # tello_pose_marker_lp[2] = (1.0 / 5.0) * (temp[2] + pxs[3] + pxs[2] + pxs[1] + pxs[0])
        tello_pose_marker_lp[4] = (1.0 / 5.0) * (temp[4] + pyaws[3] + pyaws[2] + pyaws[1] + pyaws[0])
        
        # pxs[0] = pxs[1]
        # pxs[1] = pxs[2]
        # pxs[2] = pxs[3]
        # pxs[3] = temp[0]

        # pzs[0] = pzs[1]
        # pzs[1] = pzs[2]
        # pzs[2] = pzs[3]
        # pzs[3] = temp[2]

        pyaws[0] = pyaws[1]
        pyaws[1] = pyaws[2]
        pyaws[2] = pyaws[3]
        pyaws[3] = temp[4]

rospy.Subscriber("tello_pose_marker", numpy_msg(Floats), callback=get_marker_message)

while not rospy.is_shutdown():
    pub_pose_marker_lp.publish(tello_pose_marker_lp)
    rate.sleep()