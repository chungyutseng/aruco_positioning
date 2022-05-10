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
from geometry_msgs.msg import Twist
from tello_driver.msg import TelloStatus

rospy.init_node("aruco_positioning", anonymous=True)

rate = rospy.Rate(15)

small_marker = 0.0

my_namespace=rospy.get_namespace()

if my_namespace=="/drone1/":
    calib_path = ""
    camera_matrix = np.loadtxt('/home/chungyu/.ros/cameraMatrix_C.txt', delimiter = ',')
    camera_distortion = np.loadtxt('/home/chungyu/.ros/cameraDistortion_C.txt', delimiter = ',')

if my_namespace=="/drone2/":
    calib_path = ""
    camera_matrix = np.loadtxt('/home/chungyu/.ros/cameraMatrix_B.txt', delimiter = ',')
    camera_distortion = np.loadtxt('/home/chungyu/.ros/cameraDistortion_B.txt', delimiter = ',')

R_flip = np.zeros((3, 3), dtype = np.float32)
R_flip[0, 0] = 1
R_flip[1, 2] = -1
R_flip[2, 1] = 1
font = cv2.FONT_HERSHEY_TRIPLEX

tello_pose_marker = np.zeros((6,), dtype=np.float32)

cmd_vel = Twist()

marker_detected_flag = 0.0

transformation_array_c2m = np.zeros((16,), dtype=np.float32)

battery_percentage = 0.0

img_append = cv2.imread("/home/chungyu/.ros/append.png")
img_append = cv2.resize(img_append, (300, 720), interpolation=cv2.INTER_AREA)

current_pose = np.zeros((10,), dtype=np.float32)
desired_pose = np.zeros((4,), dtype=np.float32)

aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
parameters = aruco.DetectorParameters_create()

if my_namespace=="/drone1/":
    board_ids = np.array([[0], [1], [2], [3]], dtype = np.int32)
    board_corners = [np.array([[0.0, 1.5, 1.2], [0.2, 1.5, 1.2], [0.2, 1.5, 1.0], [0.0, 1.5, 1.0]], dtype = np.float32), 
                    np.array([[0.2815, 1.5, 1.2], [0.4815, 1.5, 1.2], [0.4815, 1.5, 1.0], [0.2815, 1.5, 1.0]], dtype = np.float32),
                    np.array([[0.0, 1.5, 0.972], [0.2, 1.5, 0.972], [0.2, 1.5, 0.772], [0.0, 1.5, 0.772]], dtype = np.float32),
                    np.array([[0.0, 1.5, 1.45], [0.2, 1.5, 1.45], [0.2, 1.5, 1.25], [0.0, 1.5, 1.25]], dtype = np.float32)] # clockwise, beginning from the top-left corner

if my_namespace=="/drone2/":
    if small_marker == 1.0:
        board_ids = np.array([[11]], dtype = np.int32)
        board_corners = [np.array([[0.0, 0.0, 0.09], [0.09, 0.0, 0.09], [0.09, 0.0, 0.0], [0.0, 0.0, 0.0]], dtype = np.float32)] # clockwise, beginning from the top-left corner
    else:
        board_ids = np.array([[0], [1], [2], [3]], dtype = np.int32)
        board_corners = [np.array([[0.0, 1.5, 1.2], [0.2, 1.5, 1.2], [0.2, 1.5, 1.0], [0.0, 1.5, 1.0]], dtype = np.float32), 
                        np.array([[0.2815, 1.5, 1.2], [0.4815, 1.5, 1.2], [0.4815, 1.5, 1.0], [0.2815, 1.5, 1.0]], dtype = np.float32),
                        np.array([[0.0, 1.5, 0.972], [0.2, 1.5, 0.972], [0.2, 1.5, 0.772], [0.0, 1.5, 0.772]], dtype = np.float32),
                        np.array([[0.0, 1.5, 1.45], [0.2, 1.5, 1.45], [0.2, 1.5, 1.25], [0.0, 1.5, 1.25]], dtype = np.float32)] # clockwise, beginning from the top-left corner

board = aruco.Board_create(board_corners, aruco_dict, board_ids)

pub_marker_detected_flag = rospy.Publisher("marker_detected", Float32, queue_size=10)
pub_transformation_array_positioning = rospy.Publisher('transformation_array_positioning', numpy_msg(Floats), queue_size=10)
pub_pose_marker = rospy.Publisher('tello_pose_marker', numpy_msg(Floats), queue_size=10)

def get_desired_pose(data):
    global desired_pose
    desired_pose = data.data

def get_kf_position(data):
    global current_pose
    current_pose = data.data

def get_battery_percentage(data):
    global battery_percentage
    battery_percentage = data.battery_percentage

def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

def rotationMatrixToEulerAngles(R):
    assert(isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])    # psi: X
        y = math.atan2(-R[2, 0], sy)        # theta: Y
        z = math.atan2(R[1, 0], R[0, 0])    # theta: Z 
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])

#########################################################
# '''
# phi -> theta -> psi euler angle R_z(phi) * R_y(theta) * R_x(psi) first multiply first rotate
# '''
#########################################################

def get_cmd_vel(data):
    global cmd_vel
    cmd_vel = data

def get_battery_percentage(data):
    global battery_percentage
    battery_percentage = data.battery_percentage

def convert_color_image(ros_image):
    global tello_pose_marker
    global marker_detected_flag
    global transformation_array_c2m
    bridge = CvBridge()
    try:
        color_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")

        color_image_append = np.hstack((color_image, img_append))

        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(gray_image, aruco_dict, parameters = parameters)

        if ids is None:
            ids = np.array([[-1], [-1]], dtype=np.float32)

        if my_namespace=="/drone1/":
            if (np.any(ids[:] == 0) or np.any(ids[:] == 1) or np.any(ids[:] == 2) or np.any(ids[:] == 3)):
                marker_detected_flag = 1.0

                retval, rvec, tvec = aruco.estimatePoseBoard(corners, ids, board, camera_matrix, camera_distortion, None, None)

                R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
                R_tc = R_ct.T

                pos_camera = -R_tc * np.matrix(tvec)

                transformation_array_c2m[0] = R_tc[0, 0]
                transformation_array_c2m[1] = R_tc[0, 1]
                transformation_array_c2m[2] = R_tc[0, 2]
                transformation_array_c2m[3] = pos_camera[0] - 0.0

                transformation_array_c2m[4] = R_tc[1, 0]
                transformation_array_c2m[5] = R_tc[1, 1]
                transformation_array_c2m[6] = R_tc[1, 2]
                transformation_array_c2m[7] = pos_camera[1] - 1.5

                transformation_array_c2m[8] = R_tc[2, 0]
                transformation_array_c2m[9] = R_tc[2, 1]
                transformation_array_c2m[10] = R_tc[2, 2]
                transformation_array_c2m[11] = pos_camera[2] - 1.0

                transformation_array_c2m[12] = 0.0
                transformation_array_c2m[13] = 0.0
                transformation_array_c2m[14] = 0.0
                transformation_array_c2m[15] = 1.0         

                roll_camera, pitch_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip * R_tc)
                
                roll_camera = math.degrees(roll_camera)
                pitch_camera = math.degrees(pitch_camera)
                yaw_camera = math.degrees(yaw_camera)

                x_camera = pos_camera[0] 
                y_camera = pos_camera[1] 
                z_camera = pos_camera[2]

                # x_camera, y_camera, z_camera in meter
                # roll_camera, pitch_camera, yaw_camera in degree
                tello_pose_marker[0] = x_camera
                tello_pose_marker[1] = y_camera
                tello_pose_marker[2] = z_camera
                tello_pose_marker[3] = roll_camera
                tello_pose_marker[4] = pitch_camera
                tello_pose_marker[5] = yaw_camera

                # x_camera = x_camera - x_offset
                # y_camera = y_camera - y_offset
                # z_camera = z_camera - z_offset 

                str_position = "CAMERA Position x=%4.3f y=%4.3f z=%4.3f"%(x_camera*100, y_camera*100, z_camera*100)
                str_attitude = "CAMERA Attitude roll=%4.0f pitch=%4.0f yaw=%4.0f"%(roll_camera, pitch_camera, yaw_camera)
                cmd_vel_drone = "x_vel = %4.3f y_vel = %4.3f z_vel = %4.3f yaw_vel = %4.3f"%(cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.linear.z, cmd_vel.angular.z)
                cv2.putText(color_image_append, str_position, (0, 200), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.putText(color_image_append, str_attitude, (0, 250), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.putText(color_image_append, cmd_vel_drone, (0, 300), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

                x_position_str = "x = %4.3f"%(current_pose[0]*100)
                y_position_str = "y = %4.3f"%(current_pose[1]*100)
                z_position_str = "z = %4.3f"%(current_pose[2]*100)
                yaw_angle_str = "yaw = %4.3f"%(current_pose[3])
                cv2.putText(color_image_append, str(battery_percentage), (1060, 150), font, 1.5, (0, 64, 255), 2, cv2.LINE_AA)
                cv2.putText(color_image_append, x_position_str, (1010, 260), font, 0.8, (0, 64, 255), 2, cv2.LINE_AA)
                cv2.putText(color_image_append, y_position_str, (1010, 300), font, 0.8, (0, 64, 255), 2, cv2.LINE_AA)
                cv2.putText(color_image_append, z_position_str, (1010, 340), font, 0.8, (0, 64, 255), 2, cv2.LINE_AA)
                cv2.putText(color_image_append, yaw_angle_str, (1010, 380), font, 0.8, (0, 64, 255), 2, cv2.LINE_AA)

                d_x_position_str = "x = %4.3f"%(desired_pose[0]*100)
                d_y_position_str = "y = %4.3f"%(desired_pose[1]*100)
                d_z_position_str = "z = %4.3f"%(desired_pose[2]*100)
                d_yaw_angle_str = "yaw = %4.3f"%(math.degrees(desired_pose[3]))
                cv2.putText(color_image_append, d_x_position_str, (1010, 500), font, 0.8, (0, 64, 255), 2, cv2.LINE_AA)
                cv2.putText(color_image_append, d_y_position_str, (1010, 540), font, 0.8, (0, 64, 255), 2, cv2.LINE_AA)
                cv2.putText(color_image_append, d_z_position_str, (1010, 580), font, 0.8, (0, 64, 255), 2, cv2.LINE_AA)
                cv2.putText(color_image_append, d_yaw_angle_str, (1010, 620), font, 0.8, (0, 64, 255), 2, cv2.LINE_AA)

            else:
                marker_detected_flag = 0.0
                transformation_array_c2m = np.zeros((16,), dtype=np.float32)
                str_position = "CAMERA Position x= None y= None z= None"
                str_attitude = "CAMERA Attitude roll= None pitch= None yaw= None"
                cmd_vel_drone = "x_vel = None y_vel = None z_vel = None yaw_vel = None"
                cv2.putText(color_image_append, str_position, (0, 200), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.putText(color_image_append, str_attitude, (0, 250), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.putText(color_image_append, cmd_vel_drone, (0, 300), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

                x_position_str = "x = %4.3f"%(current_pose[0]*100)
                y_position_str = "y = %4.3f"%(current_pose[1]*100)
                z_position_str = "z = %4.3f"%(current_pose[2]*100)
                yaw_angle_str = "yaw = %4.3f"%(current_pose[3])
                cv2.putText(color_image_append, str(battery_percentage), (1060, 150), font, 1.5, (0, 64, 255), 2, cv2.LINE_AA)
                cv2.putText(color_image_append, x_position_str, (1010, 260), font, 0.8, (0, 64, 255), 2, cv2.LINE_AA)
                cv2.putText(color_image_append, y_position_str, (1010, 300), font, 0.8, (0, 64, 255), 2, cv2.LINE_AA)
                cv2.putText(color_image_append, z_position_str, (1010, 340), font, 0.8, (0, 64, 255), 2, cv2.LINE_AA)
                cv2.putText(color_image_append, yaw_angle_str, (1010, 380), font, 0.8, (0, 64, 255), 2, cv2.LINE_AA)

                d_x_position_str = "x = %4.3f"%(desired_pose[0]*100)
                d_y_position_str = "y = %4.3f"%(desired_pose[1]*100)
                d_z_position_str = "z = %4.3f"%(desired_pose[2]*100)
                d_yaw_angle_str = "yaw = %4.3f"%(math.degrees(desired_pose[3]))
                cv2.putText(color_image_append, d_x_position_str, (1010, 500), font, 0.8, (0, 64, 255), 2, cv2.LINE_AA)
                cv2.putText(color_image_append, d_y_position_str, (1010, 540), font, 0.8, (0, 64, 255), 2, cv2.LINE_AA)
                cv2.putText(color_image_append, d_z_position_str, (1010, 580), font, 0.8, (0, 64, 255), 2, cv2.LINE_AA)
                cv2.putText(color_image_append, d_yaw_angle_str, (1010, 620), font, 0.8, (0, 64, 255), 2, cv2.LINE_AA)

        if my_namespace=="/drone2/":
            if small_marker == 1.0:
                if (np.any(ids[:] == 11)):
                # if (np.any(ids[:] == 10) or np.any(ids[:] == 11)):
                    marker_detected_flag = 1.0

                    retval, rvec, tvec = aruco.estimatePoseBoard(corners, ids, board, camera_matrix, camera_distortion, None, None)

                    R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
                    R_tc = R_ct.T

                    pos_camera = -R_tc * np.matrix(tvec)

                    transformation_array_c2m[0] = R_tc[0, 0]
                    transformation_array_c2m[1] = R_tc[0, 1]
                    transformation_array_c2m[2] = R_tc[0, 2]
                    transformation_array_c2m[3] = pos_camera[0] - 0.0

                    transformation_array_c2m[4] = R_tc[1, 0]
                    transformation_array_c2m[5] = R_tc[1, 1]
                    transformation_array_c2m[6] = R_tc[1, 2]
                    transformation_array_c2m[7] = pos_camera[1] - 1.5

                    transformation_array_c2m[8] = R_tc[2, 0]
                    transformation_array_c2m[9] = R_tc[2, 1]
                    transformation_array_c2m[10] = R_tc[2, 2]
                    transformation_array_c2m[11] = pos_camera[2] - 1.0

                    transformation_array_c2m[12] = 0.0
                    transformation_array_c2m[13] = 0.0
                    transformation_array_c2m[14] = 0.0
                    transformation_array_c2m[15] = 1.0         

                    roll_camera, pitch_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip * R_tc)
                    
                    roll_camera = math.degrees(roll_camera)
                    pitch_camera = math.degrees(pitch_camera)
                    yaw_camera = math.degrees(yaw_camera)

                    x_camera = pos_camera[0] 
                    y_camera = pos_camera[1] 
                    z_camera = pos_camera[2]

                    # x_camera, y_camera, z_camera in meter
                    # roll_camera, pitch_camera, yaw_camera in degree
                    tello_pose_marker[0] = x_camera
                    tello_pose_marker[1] = y_camera
                    tello_pose_marker[2] = z_camera
                    tello_pose_marker[3] = roll_camera
                    tello_pose_marker[4] = pitch_camera
                    tello_pose_marker[5] = yaw_camera

                    # x_camera = x_camera - x_offset
                    # y_camera = y_camera - y_offset
                    # z_camera = z_camera - z_offset 

                    str_position = "CAMERA Position x=%4.3f y=%4.3f z=%4.3f"%(x_camera*100, y_camera*100, z_camera*100)
                    str_attitude = "CAMERA Attitude roll=%4.0f pitch=%4.0f yaw=%4.0f"%(roll_camera, pitch_camera, yaw_camera)
                    cmd_vel_drone = "x_vel = %4.3f y_vel = %4.3f z_vel = %4.3f yaw_vel = %4.3f"%(cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.linear.z, cmd_vel.angular.z)
                    cv2.putText(color_image, str_position, (0, 200), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                    cv2.putText(color_image, str_attitude, (0, 250), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                    cv2.putText(color_image, cmd_vel_drone, (0, 300), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

                    x_position_str = "x = %4.3f"%(current_pose[0]*100)
                    y_position_str = "y = %4.3f"%(current_pose[1]*100)
                    z_position_str = "z = %4.3f"%(current_pose[2]*100)
                    yaw_angle_str = "yaw = %4.3f"%(current_pose[3])
                    cv2.putText(color_image_append, str(battery_percentage), (1060, 150), font, 1.5, (0, 64, 255), 2, cv2.LINE_AA)
                    cv2.putText(color_image_append, x_position_str, (1010, 260), font, 0.8, (0, 64, 255), 2, cv2.LINE_AA)
                    cv2.putText(color_image_append, y_position_str, (1010, 300), font, 0.8, (0, 64, 255), 2, cv2.LINE_AA)
                    cv2.putText(color_image_append, z_position_str, (1010, 340), font, 0.8, (0, 64, 255), 2, cv2.LINE_AA)
                    cv2.putText(color_image_append, yaw_angle_str, (1010, 380), font, 0.8, (0, 64, 255), 2, cv2.LINE_AA)

                    d_x_position_str = "x = %4.3f"%(desired_pose[0]*100)
                    d_y_position_str = "y = %4.3f"%(desired_pose[1]*100)
                    d_z_position_str = "z = %4.3f"%(desired_pose[2]*100)
                    d_yaw_angle_str = "yaw = %4.3f"%(math.degrees(desired_pose[3]))
                    cv2.putText(color_image_append, d_x_position_str, (1010, 500), font, 0.8, (0, 64, 255), 2, cv2.LINE_AA)
                    cv2.putText(color_image_append, d_y_position_str, (1010, 540), font, 0.8, (0, 64, 255), 2, cv2.LINE_AA)
                    cv2.putText(color_image_append, d_z_position_str, (1010, 580), font, 0.8, (0, 64, 255), 2, cv2.LINE_AA)
                    cv2.putText(color_image_append, d_yaw_angle_str, (1010, 620), font, 0.8, (0, 64, 255), 2, cv2.LINE_AA)

                else:
                    marker_detected_flag = 0.0
                    transformation_array_c2m = np.zeros((16,), dtype=np.float32)
                    str_position = "CAMERA Position x= None y= None z= None"
                    str_attitude = "CAMERA Attitude roll= None pitch= None yaw= None"
                    cmd_vel_drone = "x_vel = None y_vel = None z_vel = None yaw_vel = None"
                    cv2.putText(color_image, str_position, (0, 200), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                    cv2.putText(color_image, str_attitude, (0, 250), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                    cv2.putText(color_image, cmd_vel_drone, (0, 300), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

                    x_position_str = "x = %4.3f"%(current_pose[0]*100)
                    y_position_str = "y = %4.3f"%(current_pose[1]*100)
                    z_position_str = "z = %4.3f"%(current_pose[2]*100)
                    yaw_angle_str = "yaw = %4.3f"%(current_pose[3])
                    cv2.putText(color_image_append, str(battery_percentage), (1060, 150), font, 1.5, (0, 64, 255), 2, cv2.LINE_AA)
                    cv2.putText(color_image_append, x_position_str, (1010, 260), font, 0.8, (0, 64, 255), 2, cv2.LINE_AA)
                    cv2.putText(color_image_append, y_position_str, (1010, 300), font, 0.8, (0, 64, 255), 2, cv2.LINE_AA)
                    cv2.putText(color_image_append, z_position_str, (1010, 340), font, 0.8, (0, 64, 255), 2, cv2.LINE_AA)
                    cv2.putText(color_image_append, yaw_angle_str, (1010, 380), font, 0.8, (0, 64, 255), 2, cv2.LINE_AA)

                    d_x_position_str = "x = %4.3f"%(desired_pose[0]*100)
                    d_y_position_str = "y = %4.3f"%(desired_pose[1]*100)
                    d_z_position_str = "z = %4.3f"%(desired_pose[2]*100)
                    d_yaw_angle_str = "yaw = %4.3f"%(math.degrees(desired_pose[3]))
                    cv2.putText(color_image_append, d_x_position_str, (1010, 500), font, 0.8, (0, 64, 255), 2, cv2.LINE_AA)
                    cv2.putText(color_image_append, d_y_position_str, (1010, 540), font, 0.8, (0, 64, 255), 2, cv2.LINE_AA)
                    cv2.putText(color_image_append, d_z_position_str, (1010, 580), font, 0.8, (0, 64, 255), 2, cv2.LINE_AA)
                    cv2.putText(color_image_append, d_yaw_angle_str, (1010, 620), font, 0.8, (0, 64, 255), 2, cv2.LINE_AA)
            else:
                if (np.any(ids[:] == 0) or np.any(ids[:] == 1) or np.any(ids[:] == 2) or np.any(ids[:] == 3)):
                    marker_detected_flag = 1.0

                    retval, rvec, tvec = aruco.estimatePoseBoard(corners, ids, board, camera_matrix, camera_distortion, None, None)

                    R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
                    R_tc = R_ct.T

                    pos_camera = -R_tc * np.matrix(tvec)

                    transformation_array_c2m[0] = R_tc[0, 0]
                    transformation_array_c2m[1] = R_tc[0, 1]
                    transformation_array_c2m[2] = R_tc[0, 2]
                    transformation_array_c2m[3] = pos_camera[0] - 0.0

                    transformation_array_c2m[4] = R_tc[1, 0]
                    transformation_array_c2m[5] = R_tc[1, 1]
                    transformation_array_c2m[6] = R_tc[1, 2]
                    transformation_array_c2m[7] = pos_camera[1] - 1.5

                    transformation_array_c2m[8] = R_tc[2, 0]
                    transformation_array_c2m[9] = R_tc[2, 1]
                    transformation_array_c2m[10] = R_tc[2, 2]
                    transformation_array_c2m[11] = pos_camera[2] - 1.0

                    transformation_array_c2m[12] = 0.0
                    transformation_array_c2m[13] = 0.0
                    transformation_array_c2m[14] = 0.0
                    transformation_array_c2m[15] = 1.0         

                    roll_camera, pitch_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip * R_tc)
                    
                    roll_camera = math.degrees(roll_camera)
                    pitch_camera = math.degrees(pitch_camera)
                    yaw_camera = math.degrees(yaw_camera)

                    x_camera = pos_camera[0] 
                    y_camera = pos_camera[1] 
                    z_camera = pos_camera[2]

                    # x_camera, y_camera, z_camera in meter
                    # roll_camera, pitch_camera, yaw_camera in degree
                    tello_pose_marker[0] = x_camera
                    tello_pose_marker[1] = y_camera
                    tello_pose_marker[2] = z_camera
                    tello_pose_marker[3] = roll_camera
                    tello_pose_marker[4] = pitch_camera
                    tello_pose_marker[5] = yaw_camera

                    # x_camera = x_camera - x_offset
                    # y_camera = y_camera - y_offset
                    # z_camera = z_camera - z_offset 

                    str_position = "CAMERA Position x=%4.3f y=%4.3f z=%4.3f"%(x_camera*100, y_camera*100, z_camera*100)
                    str_attitude = "CAMERA Attitude roll=%4.0f pitch=%4.0f yaw=%4.0f"%(roll_camera, pitch_camera, yaw_camera)
                    cmd_vel_drone = "x_vel = %4.3f y_vel = %4.3f z_vel = %4.3f yaw_vel = %4.3f"%(cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.linear.z, cmd_vel.angular.z)
                    cv2.putText(color_image_append, str_position, (0, 200), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                    cv2.putText(color_image_append, str_attitude, (0, 250), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                    cv2.putText(color_image_append, cmd_vel_drone, (0, 300), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

                    x_position_str = "x = %4.3f"%(current_pose[0]*100)
                    y_position_str = "y = %4.3f"%(current_pose[1]*100)
                    z_position_str = "z = %4.3f"%(current_pose[2]*100)
                    yaw_angle_str = "yaw = %4.3f"%(current_pose[3])
                    cv2.putText(color_image_append, str(battery_percentage), (1060, 150), font, 1.5, (0, 64, 255), 2, cv2.LINE_AA)
                    cv2.putText(color_image_append, x_position_str, (1010, 260), font, 0.8, (0, 64, 255), 2, cv2.LINE_AA)
                    cv2.putText(color_image_append, y_position_str, (1010, 300), font, 0.8, (0, 64, 255), 2, cv2.LINE_AA)
                    cv2.putText(color_image_append, z_position_str, (1010, 340), font, 0.8, (0, 64, 255), 2, cv2.LINE_AA)
                    cv2.putText(color_image_append, yaw_angle_str, (1010, 380), font, 0.8, (0, 64, 255), 2, cv2.LINE_AA)

                    d_x_position_str = "x = %4.3f"%(desired_pose[0]*100)
                    d_y_position_str = "y = %4.3f"%(desired_pose[1]*100)
                    d_z_position_str = "z = %4.3f"%(desired_pose[2]*100)
                    d_yaw_angle_str = "yaw = %4.3f"%(math.degrees(desired_pose[3]))
                    cv2.putText(color_image_append, d_x_position_str, (1010, 500), font, 0.8, (0, 64, 255), 2, cv2.LINE_AA)
                    cv2.putText(color_image_append, d_y_position_str, (1010, 540), font, 0.8, (0, 64, 255), 2, cv2.LINE_AA)
                    cv2.putText(color_image_append, d_z_position_str, (1010, 580), font, 0.8, (0, 64, 255), 2, cv2.LINE_AA)
                    cv2.putText(color_image_append, d_yaw_angle_str, (1010, 620), font, 0.8, (0, 64, 255), 2, cv2.LINE_AA)

                else:
                    marker_detected_flag = 0.0
                    transformation_array_c2m = np.zeros((16,), dtype=np.float32)
                    str_position = "CAMERA Position x= None y= None z= None"
                    str_attitude = "CAMERA Attitude roll= None pitch= None yaw= None"
                    cmd_vel_drone = "x_vel = None y_vel = None z_vel = None yaw_vel = None"
                    cv2.putText(color_image_append, str_position, (0, 200), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                    cv2.putText(color_image_append, str_attitude, (0, 250), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                    cv2.putText(color_image_append, cmd_vel_drone, (0, 300), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

                    x_position_str = "x = %4.3f"%(current_pose[0]*100)
                    y_position_str = "y = %4.3f"%(current_pose[1]*100)
                    z_position_str = "z = %4.3f"%(current_pose[2]*100)
                    yaw_angle_str = "yaw = %4.3f"%(current_pose[3])
                    cv2.putText(color_image_append, str(battery_percentage), (1060, 150), font, 1.5, (0, 64, 255), 2, cv2.LINE_AA)
                    cv2.putText(color_image_append, x_position_str, (1010, 260), font, 0.8, (0, 64, 255), 2, cv2.LINE_AA)
                    cv2.putText(color_image_append, y_position_str, (1010, 300), font, 0.8, (0, 64, 255), 2, cv2.LINE_AA)
                    cv2.putText(color_image_append, z_position_str, (1010, 340), font, 0.8, (0, 64, 255), 2, cv2.LINE_AA)
                    cv2.putText(color_image_append, yaw_angle_str, (1010, 380), font, 0.8, (0, 64, 255), 2, cv2.LINE_AA)

                    d_x_position_str = "x = %4.3f"%(desired_pose[0]*100)
                    d_y_position_str = "y = %4.3f"%(desired_pose[1]*100)
                    d_z_position_str = "z = %4.3f"%(desired_pose[2]*100)
                    d_yaw_angle_str = "yaw = %4.3f"%(math.degrees(desired_pose[3]))
                    cv2.putText(color_image_append, d_x_position_str, (1010, 500), font, 0.8, (0, 64, 255), 2, cv2.LINE_AA)
                    cv2.putText(color_image_append, d_y_position_str, (1010, 540), font, 0.8, (0, 64, 255), 2, cv2.LINE_AA)
                    cv2.putText(color_image_append, d_z_position_str, (1010, 580), font, 0.8, (0, 64, 255), 2, cv2.LINE_AA)
                    cv2.putText(color_image_append, d_yaw_angle_str, (1010, 620), font, 0.8, (0, 64, 255), 2, cv2.LINE_AA)

        cv2.namedWindow(my_namespace)
        cv2.imshow(my_namespace, color_image_append)
        cv2.waitKey(10)
        
    except CvBridgeError as e:
        print(e)

rospy.Subscriber("raw_image", Image, callback=convert_color_image)
rospy.Subscriber("tello/cmd_vel", Twist, callback=get_cmd_vel)
rospy.Subscriber("tello/status", TelloStatus, callback=get_battery_percentage)
rospy.Subscriber('tello_pose_kf', numpy_msg(Floats), callback=get_kf_position)
rospy.Subscriber('desired_pose', numpy_msg(Floats), callback=get_desired_pose)

while not rospy.is_shutdown():
    if marker_detected_flag == 1.0:
        pub_pose_marker.publish(tello_pose_marker)
    pub_marker_detected_flag.publish(marker_detected_flag)
    pub_transformation_array_positioning.publish(transformation_array_c2m)
    rate.sleep()