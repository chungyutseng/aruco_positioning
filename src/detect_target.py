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

camera_matrix = np.loadtxt('/home/chungyu/.ros/cameraMatrix.txt', delimiter = ',')
camera_distortion = np.loadtxt('/home/chungyu/.ros/cameraDistortion.txt', delimiter = ',')

transformation_array_t2c = np.zeros((16,), dtype=np.float32)

aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
parameters = aruco.DetectorParameters_create()
board_ids = np.array([[1]], dtype = np.int32)
board_corners = [np.array([[0.0, 0.0, 0.2], [0.2, 0.0, 0.2], [0.2, 0.0, 0.0], [0.0, 0.0, 0.0]], dtype = np.float32)] # clockwise, beginning from the bottom-left corner
board = aruco.Board_create(board_corners, aruco_dict, board_ids)

target_detected_flag = 0.0

pub_target_detected_flag = rospy.Publisher("/target_detected", Float32, queue_size=10)
pub_transformation_array_target = rospy.Publisher('/transformation_array_target', numpy_msg(Floats), queue_size=10)

rospy.init_node("detect_target", anonymous=True)

def convert_color_image(ros_image):
    global transformation_array_t2c
    global target_detected_flag
    bridge = CvBridge()
    try:
        color_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(gray_image, aruco_dict, parameters = parameters)

        if ids is None:
            ids = np.array([[-1], [-1]], dtype=np.float32)

        if (np.any(ids[:] == 1)):
            target_detected_flag = 1.0
            retval, rvec, tvec = aruco.estimatePoseBoard(corners, ids, board, camera_matrix, camera_distortion, None, None)
            R_t2c = np.matrix(cv2.Rodrigues(rvec)[0])
            t_t2c = np.matrix(tvec)

            transformation_array_t2c[0] = R_t2c[0, 0]
            transformation_array_t2c[1] = R_t2c[0, 1]
            transformation_array_t2c[2] = R_t2c[0, 2]
            transformation_array_t2c[3] = t_t2c[0]

            transformation_array_t2c[4] = R_t2c[1, 0]
            transformation_array_t2c[5] = R_t2c[1, 1]
            transformation_array_t2c[6] = R_t2c[1, 2]
            transformation_array_t2c[7] = t_t2c[1]

            transformation_array_t2c[8] = R_t2c[2, 0]
            transformation_array_t2c[9] = R_t2c[2, 1]
            transformation_array_t2c[10] = R_t2c[2, 2]
            transformation_array_t2c[11] = t_t2c[2]

            transformation_array_t2c[12] = 0.0
            transformation_array_t2c[13] = 0.0
            transformation_array_t2c[14] = 0.0
            transformation_array_t2c[15] = 1.0

        else:
            target_detected_flag = 0.0
            transformation_array_t2c = np.zeros((16,), dtype=np.float32)

    except CvBridgeError as e:
        print(e)

rospy.Subscriber("/raw_image", Image, callback=convert_color_image, queue_size=10)

while not rospy.is_shutdown():
    rate = rospy.Rate(15)
    pub_target_detected_flag.publish(target_detected_flag)
    pub_transformation_array_target.publish(transformation_array_t2c)