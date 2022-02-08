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

rospy.init_node("aruco_positioning", anonymous=True)

rate = rospy.Rate(15)

marker_size = 0.2
marker_height = 1.01
marker_offset = 0.08155 # it is the distance between marker 1 and marker 2

calib_path = ""
camera_matrix = np.loadtxt('/home/chungyu/.ros/cameraMatrix.txt', delimiter = ',')
camera_distortion = np.loadtxt('/home/chungyu/.ros/cameraDistortion.txt', delimiter = ',')

R_flip = np.zeros((3, 3), dtype = np.float32)
R_flip[0, 0] = 1
R_flip[1, 2] = -1
R_flip[2, 1] = 1
font = cv2.FONT_HERSHEY_PLAIN

tello_pose_marker = np.zeros((6,), dtype=np.float32)

cmd_vel = Twist()

# roll_camera = 0.0
# pitch_camera = 0.0
# yaw_camera = 0.0
# x_camera = 0.0
# y_camera = 0.0
# z_camera = 0.0

# cmd_vel_linear_x = 0.0
# cmd_vel_linear_y = 0.0
# cmd_vel_linear_z = 0.0
# cmd_vel_angular_z = 0.0

marker_detected_flag = 0.0

transformation_array_c2m = np.zeros((16,), dtype=np.float32)

aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
parameters = aruco.DetectorParameters_create()
# board_ids = np.array([[0], [1]], dtype = np.int32)
# board_corners = [np.array([[0.0, 1.5, marker_height + marker_size], [marker_size, 1.5, marker_height + marker_size], [marker_size, 1.5, marker_height], [0.0, 1.5, marker_height]], dtype = np.float32), 
#                 np.array([[marker_size + marker_offset, 1.5, marker_height + marker_size], [2 * marker_size + marker_offset, 1.5, marker_height + marker_size], [2 * marker_size + marker_offset, 1.5, marker_height], [marker_size + marker_offset, 1.5, marker_height]], dtype = np.float32)] # clockwise, beginning from the top-left corner
board_ids = np.array([[0], [1]], dtype = np.int32)
board_corners = [np.array([[0.0, 1.5, 1.2], [0.2, 1.5, 1.2], [0.2, 1.5, 1.0], [0.0, 1.5, 1.0]], dtype = np.float32), 
                np.array([[0.28155, 1.5, 1.2], [0.48155, 1.5, 1.2], [0.48155, 1.5, 1.0], [0.28155, 1.5, 1.0]], dtype = np.float32)] # clockwise, beginning from the top-left corner
board = aruco.Board_create(board_corners, aruco_dict, board_ids)

# pub_x = rospy.Publisher("/x", Float32, queue_size=10)
# pub_y = rospy.Publisher("/y", Float32, queue_size=10)
# pub_z = rospy.Publisher("/z", Float32, queue_size=10)
# pub_roll = rospy.Publisher("/roll", Float32, queue_size=10)
# pub_pitch = rospy.Publisher("/pitch", Float32, queue_size=10)
# pub_yaw = rospy.Publisher("/yaw", Float32, queue_size=10)
pub_marker_detected_flag = rospy.Publisher("/marker_detected", Float32, queue_size=10)
pub_transformation_array_positioning = rospy.Publisher('/transformation_array_positioning', numpy_msg(Floats), queue_size=10)
pub_pose_marker = rospy.Publisher('/tello_pose_marker', numpy_msg(Floats), queue_size=10)
# pub_pose_marker = rospy.Publisher('/tello_pose_marker', Twist, queue_size=10)

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

# def get_cmd_vel_linear_x(data):
#     global cmd_vel_linear_x
#     cmd_vel_linear_x = data.data

# def get_cmd_vel_linear_y(data):
#     global cmd_vel_linear_y
#     cmd_vel_linear_y = data.data

# def get_cmd_vel_linear_z(data):
#     global cmd_vel_linear_z
#     cmd_vel_linear_z = data.data

# def get_cmd_vel_angular_z(data):
#     global cmd_vel_angular_z
#     cmd_vel_angular_z = data.data

def convert_color_image(ros_image):
    # global roll_camera, pitch_camera, yaw_camera, x_camera, y_camera, z_camera
    # global cmd_vel_linear_x, cmd_vel_linear_y, cmd_vel_linear_z, cmd_vel_angular_z
    global tello_pose_marker
    global marker_detected_flag
    global transformation_array_c2m
    bridge = CvBridge()
    try:
        color_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(gray_image, aruco_dict, parameters = parameters)

        if ids is None:
            ids = np.array([[-1], [-1]], dtype=np.float32)

        if (np.any(ids[:] == 0) or np.any(ids[:] == 1)):
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
            transformation_array_c2m[11] = pos_camera[2] - marker_height

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

        else:
            marker_detected_flag = 0.0
            transformation_array_c2m = np.zeros((16,), dtype=np.float32)
            str_position = "CAMERA Position x= None y= None z= None"
            str_attitude = "CAMERA Attitude roll= None pitch= None yaw= None"
            cmd_vel_drone = "x_vel = None y_vel = None z_vel = None yaw_vel = None"
            cv2.putText(color_image, str_position, (0, 200), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(color_image, str_attitude, (0, 250), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(color_image, cmd_vel_drone, (0, 300), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

        cv2.namedWindow("Color")
        cv2.imshow("Color", color_image)
        cv2.waitKey(10)
        
    except CvBridgeError as e:
        print(e)

rospy.Subscriber("/raw_image", Image, callback=convert_color_image, queue_size=10)
rospy.Subscriber("/tello/cmd_vel", Twist, callback=get_cmd_vel)
# rospy.Subscriber("/cmd_vel_linear_x", Float32, callback=get_cmd_vel_linear_x, queue_size=10)
# rospy.Subscriber("/cmd_vel_linear_y", Float32, callback=get_cmd_vel_linear_y, queue_size=10)
# rospy.Subscriber("/cmd_vel_linear_z", Float32, callback=get_cmd_vel_linear_z, queue_size=10)
# rospy.Subscriber("/cmd_vel_angular_z", Float32, callback=get_cmd_vel_angular_z, queue_size=10)

while not rospy.is_shutdown():
    if marker_detected_flag == 1.0:
        # pub_x.publish(x_camera)
        # pub_y.publish(y_camera)
        # pub_z.publish(z_camera)
        # pub_roll.publish(roll_camera)
        # pub_pitch.publish(pitch_camera)
        # pub_yaw.publish(yaw_camera)
        # tello_pose_marker = Twist()
        # tello_pose_marker.linear.x = x_camera
        # tello_pose_marker.linear.y = y_camera
        # tello_pose_marker.linear.z = z_camera
        # tello_pose_marker.angular.x = roll_camera
        # tello_pose_marker.angular.y = pitch_camera
        # tello_pose_marker.angular.z = yaw_camera
        pub_pose_marker.publish(tello_pose_marker)
    pub_marker_detected_flag.publish(marker_detected_flag)
    pub_transformation_array_positioning.publish(transformation_array_c2m)
    rate.sleep()