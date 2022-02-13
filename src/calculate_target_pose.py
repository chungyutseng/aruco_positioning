#!/usr/bin/env python3
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import cv2
import cv2.aruco as aruco
import math
from std_msgs.msg import Float32
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

rospy.init_node("calculate_target_pose", anonymous=True)

rate = rospy.Rate(15)

transformation_array_positioning = np.zeros((16,), dtype=np.float32)
transformation_array_target = np.zeros((16,), dtype=np.float32)
# target_x = 0.0
# target_y = 0.0
# target_z = 0.0
# target_roll = 0.0
# target_pitch = 0.0
# target_yaw = 0.0

# x, y, z, roll, pitch, yaw
target_pose_estimated = np.zeros((6,), dtype=np.float32)

transformation_matrix_positioning = np.zeros((4, 4), dtype=np.float32)
transformation_matrix_target = np.zeros((4, 4), dtype=np.float32)
transformation_matrix_positioning_marker_to_world = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 1.5], [0.0, 0.0, 1.0, 1.0], [0.0, 0.0, 0.0, 1.0]], dtype=np.float32)

target_pose = np.zeros((4, 4), dtype=np.float32)
# target_rotation_matrix = np.zeros((3, 3), dtype=np.float32)

# pub_target_x = rospy.Publisher("/target_x", Float32, queue_size=10)
# pub_target_y = rospy.Publisher("/target_y", Float32, queue_size=10)
# pub_target_z = rospy.Publisher("/target_z", Float32, queue_size=10)
# pub_target_roll = rospy.Publisher("/target_roll", Float32, queue_size=10)
# pub_target_pitch = rospy.Publisher("/target_pitch", Float32, queue_size=10)
# pub_target_yaw = rospy.Publisher("/target_yaw", Float32, queue_size=10)
pub_target_pose_estimated = rospy.Publisher('/target_pose_estimated', numpy_msg(Floats), queue_size=10)

def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

#########################################################
# '''
# phi -> theta -> psi euler angle R_z(phi) * R_y(theta) * R_x(psi) first multiply first rotate
# '''
#########################################################
def rotationMatrixToEulerAngles(R):

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])    # psi: X
        y = math.atan2(-R[2, 0], sy)        # theta: Y
        z = math.atan2(R[1, 0], R[0, 0])    # phi: Z 
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])

def get_transformation_array_positioning(data):
    global transformation_array_positioning
    global transformation_matrix_positioning
    transformation_array_positioning = data.data
    transformation_matrix_positioning = transformation_array_positioning.reshape(4, 4)

def get_transformation_array_target(data):
    global transformation_array_target
    global transformation_matrix_target
    transformation_array_target = data.data
    transformation_matrix_target = transformation_array_target.reshape(4, 4)

def get_target_detected_flag(data):
    global target_pose
    global target_pose_estimated
    # global target_rotation_matrix
    # global target_x, target_y, target_z
    # global target_roll, target_pitch, target_yaw
    temp = np.zeros((4, 4), dtype=np.float32)
    temp = transformation_matrix_positioning_marker_to_world.dot(transformation_matrix_positioning)
    target_pose = temp.dot(transformation_matrix_target)
    if (target_pose[3, 3] != 0):
        target_pose_estimated[0] = target_pose[0, 3]
        target_pose_estimated[1] = target_pose[1, 3]
        target_pose_estimated[2] = target_pose[2, 3]
        # target_pose_estimated.linear.x = target_pose[0, 3]
        # target_pose_estimated.linear.y = target_pose[1, 3]
        # target_pose_estimated.linear.z = target_pose[2, 3]
        # target_x = target_pose[0, 3]
        # target_y = target_pose[1, 3]
        # target_z = target_pose[2, 3]
        target_rotation_matrix = target_pose[0:3, 0:3]
        target_roll, target_pitch, target_yaw = rotationMatrixToEulerAngles(target_rotation_matrix)
        target_pose_estimated[3] = math.degrees(target_roll)
        target_pose_estimated[4] = math.degrees(target_pitch)
        target_pose_estimated[5] = math.degrees(target_yaw)
        # target_roll = math.degrees(target_roll)
        # target_pitch = math.degrees(target_pitch)
        # target_yaw = math.degrees(target_yaw)

rospy.Subscriber('/transformation_array_positioning', numpy_msg(Floats), callback=get_transformation_array_positioning)
rospy.Subscriber('/transformation_array_target', numpy_msg(Floats), callback=get_transformation_array_target)
rospy.Subscriber('/target_detected', Float32, callback=get_target_detected_flag)

while not rospy.is_shutdown():
    if (target_pose[3, 3] != 0):
        pub_target_pose_estimated.publish(target_pose_estimated)
        # pub_target_x.publish(target_x)
        # pub_target_y.publish(target_y)
        # pub_target_z.publish(target_z)
        # pub_target_roll.publish(target_roll)
        # pub_target_pitch.publish(target_pitch)
        # pub_target_yaw.publish(target_yaw)
    rate.sleep()