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
desired_height = 1.1

kp = 1

vel_max_linear = 0.8
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

# marker_height = 0.8

# # pub_takeoff = rospy.Publisher('/tello/takeoff', EmptyMsg, queue_size=10)
# pub_manual_takeoff = rospy.Publisher('tello/manual_takeoff', EmptyMsg, queue_size=10)
# pub_desired_pose = rospy.Publisher('desired_pose', numpy_msg(Floats), queue_size=10)
# pub_vel = rospy.Publisher("tello/cmd_vel", Twist, queue_size=10)
# pub_VO_on_off = rospy.Publisher("VO_on_off", Float32, queue_size=10)
# pub_height = rospy.Publisher("tello_height", Float32, queue_size=10)
# # pub_desired_x = rospy.Publisher('/desired_x', Float32, queue_size=10)
# # pub_desired_y = rospy.Publisher('/desired_y', Float32, queue_size=10)
# # pub_desired_z = rospy.Publisher('/desired_z', Float32, queue_size=10)
# # pub_desired_yaw = rospy.Publisher('/desired_yaw', Float32, queue_size=10)
# # pub_control = rospy.Publisher('/controller_on', Float32, queue_size=10)

# time.sleep(3)
# pub_manual_takeoff.publish(EmptyMsg())
# time.sleep(4)

# desired_pose = np.zeros((4,), dtype=np.float32)

# height = 0.0
# desired_height = 1.1
# vel_msg = Twist()
# vel_msg.linear.x = 0.0
# vel_msg.linear.y = 0.0
# vel_msg.linear.z = 0.0
# vel_msg.angular.x = 0.0
# vel_msg.angular.y = 0.0
# vel_msg.angular.z = 0.0
# max_z_vel = 20
# p_z = 2

# marker_detected_flag = 0.0

# flag_VO = 0.0

# def get_height(msg):
#     global height
#     global vel_msg
#     height = msg.height_m
#     vel_msg.linear.z = p_z * (desired_height - height)
#     sign = functools.partial(math.copysign, 1)
#     if abs(vel_msg.linear.z) > max_z_vel:
#         vel_msg.linear.z = sign(vel_msg.linear.z) * max_z_vel

# def get_marker_detected(data):
#     global marker_detected_flag
#     marker_detected_flag = data.data

# def motion_one():
#     global desired_pose
#     desired_pose[0] = 0.0 # in meter
#     # desired_pose[1] = -1.0 # in meter
#     desired_pose[1] = 0.0 # in meter
#     # desired_pose[2] = 0.1040 # in meter
#     desired_pose[2] = 1.2 # in meter
#     desired_pose[3] = 0 * (math.pi / 180.0) # in radians

# def motion_two():
#     global flag, time_old
#     global desired_pose
#     time_now = time.time()
#     if (time_now - time_old) > 25:
#         if flag == 1:
#             flag = 0
#         else:
#             flag = 1
#         time_old = time_now
#     if flag == 0:
#         desired_pose[0] = 0.0
#         desired_pose[1] = 0.0
#         desired_pose[2] = 1.2
#         desired_pose[3] = 0.0
#     if flag == 1:
#         desired_pose[0] = 0.0
#         desired_pose[1] = -0.5
#         desired_pose[2] = 1.2
#         desired_pose[3] = 0.0

# flag = 0
# time_old = time.time()
# # time_old_VO = time.time()

# rospy.Subscriber("tello/status", TelloStatus, callback=get_height)
# rospy.Subscriber("marker_detected", Float32, callback=get_marker_detected)

# while not rospy.is_shutdown():
#     # time_now_VO = time.time()
#     if (marker_detected_flag == 1.0):
#         flag_VO = 1.0

#     if (flag_VO == 1.0):
#         pub_VO_on_off.publish(1.0)
#     else:
#         pub_VO_on_off.publish(0.0)
#         pub_vel.publish(vel_msg)
#     motion_two()
#     pub_desired_pose.publish(desired_pose)
#     pub_height.publish(height)
#     rate.sleep()