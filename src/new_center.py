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

rospy.init_node('center', anonymous=True)

rate = rospy.Rate(15)

pub_drone1_mt = rospy.Publisher("/drone1/tello/manual_takeoff", EmptyMsg, queue_size=10)
pub_drone2_mt = rospy.Publisher("/drone2/tello/manual_takeoff", EmptyMsg, queue_size=10)
pub_pc_drone1 = rospy.Publisher("/drone1/pc_on", Float32, queue_size=10)
pub_pdc_drone1 = rospy.Publisher("/drone1/pdc_on", Float32, queue_size=10)
pub_pc_drone2 = rospy.Publisher("/drone2/pc_on", Float32, queue_size=10)
pub_pdc_drone2 = rospy.Publisher("/drone2/pdc_on", Float32, queue_size=10)

time.sleep(5)

pub_drone1_mt.publish()
pub_drone2_mt.publish()

time.sleep(5)

pub_pc_drone1.publish(1.0)
pub_pdc_drone1.publish(0.0)
pub_pc_drone2.publish(1.0)
pub_pdc_drone2.publish(0.0)

time.sleep(3)

pub_pc_drone1.publish(0.0)
pub_pdc_drone1.publish(1.0)
pub_pc_drone2.publish(0.0)
pub_pdc_drone2.publish(1.0)

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