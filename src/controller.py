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
import functools

rospy.init_node('controller', anonymous=True)

x_position = 0.0
y_position = 0.0
z_position = 0.0
yaw_angle = 0.0
desired_x_position = 0.0
desired_y_position = 0.0
desired_z_position = 0.0
desired_yaw_angle = 0.0

controller_on_or_off = 0.0

kp_x = 1.5
kp_y = 1.8
kp_z = 1.5
kp_yaw = 2.5

kd_x = 0.01
kd_y = 0.01
kd_z = 0.01
kd_yaw = 0.01

previous_time_x = time.time() - time.time()
previous_time_y = time.time() - time.time()
previous_time_z = time.time() - time.time()
previous_time_yaw = time.time() - time.time()
previous_error_x = 0.0
previous_error_y = 0.0
previous_error_z = 0.0
previous_error_yaw = 0.0

count = 0.0

vel_max_linear = 1
vel_max_angular = 1

pub_vel = rospy.Publisher("/tello/cmd_vel", Twist, queue_size=10)
pub_vel_x_linear = rospy.Publisher("/cmd_vel_linear_x", Float32, queue_size=10)
pub_vel_y_linear = rospy.Publisher("/cmd_vel_linear_y", Float32, queue_size=10)
pub_vel_z_linear = rospy.Publisher("/cmd_vel_linear_z", Float32, queue_size=10)
pub_vel_z_angular = rospy.Publisher("/cmd_vel_angular_z", Float32, queue_size=10)

vel_msg = Twist()

def get_x_position(data):
    global x_position
    x_position = data.data

def get_y_position(data):
    global y_position
    y_position = data.data

def get_z_position(data):
    global z_position
    z_position = data.data

def get_yaw_angle(data):
    global yaw_angle
    yaw_angle = data.data

def get_desired_x_position(data):
    global desired_x_position
    desired_x_position = data.data

def get_desired_y_position(data):
    global desired_y_position
    desired_y_position = data.data

def get_desired_z_position(data):
    global desired_z_position
    desired_z_position = data.data

def get_desired_yaw_angle(data):
    global desired_yaw_angle
    desired_yaw_angle = data.data

def controller_on(data):
    global controller_on_or_off
    global x_position, y_position, z_position, yaw_angle, desired_x_position, desired_y_position, desired_z_position, desired_yaw_angle
    global kp_x, kp_y, kp_z, kp_yaw, kd_x, kd_y, kd_z, kd_yaw
    if data.data == 1.0:
        controller_on_or_off = 1.0
        pd_controller(x_position, y_position, z_position, yaw_angle, desired_x_position, desired_y_position, desired_z_position, desired_yaw_angle, kp_x, kp_y, kp_z, kp_yaw, kd_x, kd_y, kd_z, kd_yaw)
    else:
        controller_on_or_off = 0.0
        pd_controller(x_position, y_position, z_position, yaw_angle, desired_x_position, desired_y_position, desired_z_position, desired_yaw_angle, kp_x, kp_y, kp_z, kp_yaw, kd_x, kd_y, kd_z, kd_yaw)

'''
PD Controller
'''
def pd_controller(x_p, y_p, z_p, yaw_a, dx_p, dy_p, dz_p, dyaw_a, KP_X, KP_Y, KP_Z, KP_YAW, KD_X, KD_Y, KD_Z, KD_YAW):
    global controller_on_or_off
    global pub_vel
    global count
    global previous_time_x, previous_time_y, previous_time_z, previous_time_yaw, previous_error_x, previous_error_y, previous_error_z, previous_error_yaw
    global vel_max_linear, vel_max_angular
    global pub_vel_x_linear, pub_vel_y_linear, pub_vel_z_linear, pub_vel_z_angular
    global vel_msg

    if controller_on_or_off == 1.0:
        if count == 0.0:
            yaw_a = math.radians(yaw_a)
            A = np.array([[math.cos(-yaw_a), -math.sin(-yaw_a)],
                        [math.sin(-yaw_a), math.cos(-yaw_a)]])
            A_transpose = A.T
            now_position = np.array([[x_p], [y_p]])
            T_position = - A_transpose.dot(now_position)
            Trans_matrix = np.array([[A_transpose[0][0], A_transpose[0][1], T_position[0][0]],
                                    [A_transpose[1][0], A_transpose[1][1], T_position[1][0]], 
                                    [0                , 0                , 1              ]])
            goal_position = np.array([[dx_p], [dy_p], [1]])
            goal_wrt_drone = Trans_matrix.dot(goal_position)
            x_goal_wrt_drone = goal_wrt_drone[0][0]
            y_goal_wrt_drone = goal_wrt_drone[1][0]

            previous_error_x = x_goal_wrt_drone - 0.0
            previous_time_x = time.time()
            previous_error_y = y_goal_wrt_drone - 0.0
            previous_time_y = time.time()

            now_z_position = z_p
            previous_error_z = dz_p - z_p
            previous_time_z = time.time()

            now_yaw_angle = yaw_a
            previous_error_yaw = dyaw_a - now_yaw_angle
            previous_time_yaw = time.time()

            count = count + 1
        else:
            vel_msg = Twist()

            yaw_a = math.radians(yaw_a)
            A = np.array([[math.cos(-yaw_a), -math.sin(-yaw_a)],
                        [math.sin(-yaw_a), math.cos(-yaw_a)]])
            A_transpose = A.T
            now_position = np.array([[x_p], [y_p]])
            T_position = - A_transpose.dot(now_position)
            Trans_matrix = np.array([[A_transpose[0][0], A_transpose[0][1], T_position[0][0]],
                                    [A_transpose[1][0], A_transpose[1][1], T_position[1][0]], 
                                    [0                , 0                , 1              ]])
            goal_position = np.array([[dx_p], [dy_p], [1]])
            goal_wrt_drone = Trans_matrix.dot(goal_position)
            x_goal_wrt_drone = goal_wrt_drone[0][0]
            y_goal_wrt_drone = goal_wrt_drone[1][0]       

            now_error_x = x_goal_wrt_drone - 0.0
            now_time_x = time.time()
            now_error_y = y_goal_wrt_drone - 0.0
            now_time_y = time.time()

            # vel_msg.linear.x = now_error_x * KP_X + ((now_error_x - previous_error_x) / (now_time_x - previous_time_x)) * KD_X
            # vel_msg.linear.y = now_error_y * KP_Y + ((now_error_y - previous_error_y) / (now_time_y - previous_time_y)) * KD_Y
            vel_msg.linear.x = now_error_x * KP_X + ((now_error_x - previous_error_x) / (1.0/15)) * KD_X
            vel_msg.linear.y = now_error_y * KP_Y + ((now_error_y - previous_error_y) / (1.0/15)) * KD_Y

            previous_error_x = now_error_x
            previous_error_y = now_error_y
            previous_time_x = now_time_x
            previous_time_y = now_time_y

            now_z_position = z_p
            now_error_z = dz_p - z_p
            now_time_z = time.time()

            # vel_msg.linear.z = now_error_z * KP_Z + ((now_error_z - previous_error_z) / (now_time_z - previous_time_z)) * KD_Z
            vel_msg.linear.z = now_error_z * KP_Z + ((now_error_z - previous_error_z) / (1.0/15)) * KD_Z

            previous_error_z = now_error_z
            previous_time_z = now_time_z

            now_yaw_angle = yaw_a
            now_error_yaw = dyaw_a - now_yaw_angle
            now_time_yaw = time.time()

            # vel_msg.angular.z = (now_error_yaw * KP_YAW + ((now_error_yaw - previous_error_yaw) / (now_time_yaw - previous_time_yaw)) * KD_YAW)
            vel_msg.angular.z = (now_error_yaw * KP_YAW + ((now_error_yaw - previous_error_yaw) / (1.0/15)) * KD_YAW)

            previous_error_yaw = now_error_yaw
            previous_time_yaw = now_time_yaw

            vel_msg.angular.x = 0
            vel_msg.angular.y = 0

            sign = functools.partial(math.copysign, 1)
            if abs(vel_msg.linear.x) > vel_max_linear:
                vel_msg.linear.x = sign(vel_msg.linear.x) * vel_max_linear
            if abs(vel_msg.linear.y) > vel_max_linear:
                vel_msg.linear.y = sign(vel_msg.linear.y) * vel_max_linear
            if abs(vel_msg.linear.z) > vel_max_linear:
                vel_msg.linear.z = sign(vel_msg.linear.z) * vel_max_linear
            if abs(vel_msg.angular.z) > vel_max_angular:
                vel_msg.angular.z = sign(vel_msg.angular.z) * vel_max_angular

            pub_vel_x_linear.publish(vel_msg.linear.x)
            pub_vel_y_linear.publish(vel_msg.linear.y)
            pub_vel_z_linear.publish(vel_msg.linear.z)
            pub_vel_z_angular.publish(vel_msg.angular.z)
            # pub_vel.publish(vel_msg)
    else:
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = 0.0
        # pub_vel.publish(vel_msg)

rospy.Subscriber("/x", Float32, callback=get_x_position)
rospy.Subscriber("/y", Float32, callback=get_y_position)
rospy.Subscriber("/z", Float32, callback=get_z_position)
rospy.Subscriber("/pitch", Float32, callback=get_yaw_angle)
rospy.Subscriber("/desired_x", Float32, callback=get_desired_x_position)
rospy.Subscriber("/desired_y", Float32, callback=get_desired_y_position)
rospy.Subscriber("/desired_z", Float32, callback=get_desired_z_position)
rospy.Subscriber("/desired_yaw", Float32, callback=get_desired_yaw_angle)
rospy.Subscriber("/controller_on", Float32, callback=controller_on)

while not rospy.is_shutdown():
    rate = rospy.Rate(15)
    pub_vel.publish(vel_msg)
    rate.sleep()



# x = -0.7
# z = 1.0
# y = 1.5-0.424

# test2
# kp_x = 1.5
# kp_y = 1.8
# kp_z = 1.5
# kp_yaw = 3.2
# pub_desired_x.publish(0.0)
# pub_desired_y.publish(0.0)
# pub_desired_z.publish(1.2)

# test3
# kp_x = 1.5
# kp_y = 1.8
# kp_z = 1.5
# kp_yaw = 2.5
# pub_desired_x.publish(0.0)
# pub_desired_y.publish(-0.3)
# pub_desired_z.publish(1.2)