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

rospy.init_node('center', anonymous=True)

rate = rospy.Rate(15)

marker_height = 1.01

pub_takeoff = rospy.Publisher('/tello/takeoff', EmptyMsg, queue_size=10)
pub_desired_pose = rospy.Publisher('/desired_pose', numpy_msg(Floats), queue_size=10)
# pub_desired_x = rospy.Publisher('/desired_x', Float32, queue_size=10)
# pub_desired_y = rospy.Publisher('/desired_y', Float32, queue_size=10)
# pub_desired_z = rospy.Publisher('/desired_z', Float32, queue_size=10)
# pub_desired_yaw = rospy.Publisher('/desired_yaw', Float32, queue_size=10)
# pub_control = rospy.Publisher('/controller_on', Float32, queue_size=10)

time.sleep(10)
pub_takeoff.publish()
time.sleep(1)

desired_pose = np.zeros((4,), dtype=np.float32)

def motion_one():
    global desired_pose
    desired_pose[0] = 0.0 # in meter
    desired_pose[1] = 0.0 # in meter
    desired_pose[2] = 1.2 # in meter
    desired_pose[3] = 0.0 * (math.pi / 180.0) # in radians

def motion_two():
    global flag, time_old
    global desired_pose
    time_now = time.time()
    if (time_now - time_old) > 15:
        if flag == 1:
            flag = 0
        else:
            flag = 1
        time_old = time_now
    if flag == 0:
        desired_pose[0] = 0.0
        desired_pose[1] = 0.0
        desired_pose[2] = 1.2
        desired_pose[3] = 0.0
    if flag == 1:
        desired_pose[0] = 0.0
        desired_pose[1] = -0.3
        desired_pose[2] = 1.2
        desired_pose[3] = 0.0

flag = 0
time_old = time.time()

while not rospy.is_shutdown():
    motion_one()
    pub_desired_pose.publish(desired_pose)
    rate.sleep()

# flag = 0
# time_old = time.time() - time.time()
# marker_detected_flag = 0.0

# def get_marker_detected_flag(data):
#     global marker_detected_flag
#     marker_detected_flag = data.data

# def takeoff():
#     global pub_takeoff
#     time.sleep(10)
#     pub_takeoff.publish()
#     time.sleep(1)

# def pub_on():
#     global pub_desired_x, pub_desired_y, pub_desired_z, pub_desired_yaw, pub_control
#     global rate
#     global marker_detected_flag
#     rospy.Subscriber("/marker_detected", Float32, callback=get_marker_detected_flag)
#     while not rospy.is_shutdown():
#         pub_desired_x.publish(0.0)
#         pub_desired_y.publish(-0.3)
#         pub_desired_z.publish(1.2)
#         pub_desired_yaw.publish(0.0)
#         if marker_detected_flag == 1.0:
#             pub_control.publish(1.0)
#         else:
#             pub_control.publish(0.0)
#         rate.sleep()

# def pub_on_1():
#     global pub_desired_x, pub_desired_y, pub_desired_z, pub_desired_yaw, pub_control
#     global rate
#     global flag, time_old
#     global marker_detected_flag
#     rospy.Subscriber("/marker_detected", Float32, callback=get_marker_detected_flag)
#     time_old = time.time()
#     while not rospy.is_shutdown():
#         time_now = time.time()
#         if (time_now - time_old) > 10:
#             if flag == 1:
#                 flag = 0
#             else:
#                 flag = 1
#             time_old = time_now
#         if flag == 0:
#             pub_desired_x.publish(0.0)
#             pub_desired_y.publish(0.0)
#             pub_desired_z.publish(1.2)
#             pub_desired_yaw.publish(0.0)
#             pub_control.publish(1.0)
#         if flag == 1:
#             pub_desired_x.publish(0.0)
#             pub_desired_y.publish(-1.3)
#             pub_desired_z.publish(1.2)
#             pub_desired_yaw.publish(0.0)
#             pub_control.publish(1.0)
#         if marker_detected_flag == 1.0:
#             pub_control.publish(1.0)
#         else:
#             pub_control.publish(0.0)
#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         takeoff()
#         pub_on()
#         # pub_on_1()
#     except rospy.ROSInterruptException:
#         pass