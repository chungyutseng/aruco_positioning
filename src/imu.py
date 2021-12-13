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

rospy.init_node('imu', anonymous=True)

def get_imu_message(imu_msg):
    

def imu():
    rospy.Subscriber("/tello/imu", ImuMsg, callback=get_imu_message, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    try:
        imu()
    except rospy.ROSInterruptException:
        pass