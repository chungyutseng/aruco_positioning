#!/usr/bin/env python3
import rospy
from std_msgs.msg import Empty as EmptyMsg
from geometry_msgs.msg import Twist
import time
from std_msgs.msg import Int8
from std_msgs.msg import Float32

pre_time = time.time() - time.time()

pub_takeoff = rospy.Publisher('tello/takeoff', EmptyMsg, queue_size=1)
pub_land = rospy.Publisher('tello/land', EmptyMsg, queue_size=1)
pub_vel_cmd = rospy.Publisher('tello/cmd_vel_QQQQQ', Twist, queue_size=1)

command = 0

marker_detected_flag = 0.0

def get_marker_detected_flag(data):
    global marker_detected_flag
    marker_detected_flag = data.data

def pub_cmd(data):
    global pre_time, command
    global pub_vel_cmd, pub_takeoff, pub_land
    pre_time = time.time() * 1000  #in milisecond
    command = data.data

def cmd():
    global pre_time, command
    global pub_vel_cmd
    global marker_detected_flag
    rospy.init_node('cmd', anonymous=True)
    rospy.Subscriber("keyboard_cmd", Int8, callback=pub_cmd)
    rospy.Subscriber("marker_detected", Float32, callback=get_marker_detected_flag)
    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():
        if marker_detected_flag == 0.0:
            now_time = time.time() * 1000
            if (now_time - pre_time) > 100:
                vel_msg = Twist()
                vel_msg.linear.x = 0
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0  
                vel_msg.angular.z = 0
                pub_vel_cmd.publish(vel_msg)
            else:
                vel_msg = Twist()
                vel_msg.linear.x = 0
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0  
                vel_msg.angular.z = 0

                # if command == 108:
                #     pub_land.publish()
                velocity = 0.5
                if command == 116:
                    pub_takeoff.publish()
                # elif command == 108:
                #     pub_land.publish()
                elif command == 119: #w forward 
                    vel_msg.linear.y = velocity
                    pub_vel_cmd.publish(vel_msg)        
                elif command == 115: #s
                    vel_msg.linear.y = -velocity
                    pub_vel_cmd.publish(vel_msg)        
                elif command == 97:  #a
                    vel_msg.linear.x = -velocity
                    pub_vel_cmd.publish(vel_msg)
                elif command == 100: #d
                    vel_msg.linear.x = velocity
                    pub_vel_cmd.publish(vel_msg)
                elif command == 113: #q rotate left, viewed from the top
                    vel_msg.angular.z = -velocity
                    pub_vel_cmd.publish(vel_msg)
                elif command == 101: #e rotate right, viewed from the top
                    vel_msg.angular.z = velocity
                    pub_vel_cmd.publish(vel_msg)
                elif command == 32: #space
                    vel_msg.linear.z = velocity
                    pub_vel_cmd.publish(vel_msg)
                elif command == 122: #z
                    vel_msg.linear.z = -velocity
                    pub_vel_cmd.publish(vel_msg)

        if command == 108:
            pub_land.publish()
        # print("QQQ\n")

        rate.sleep()

if __name__ == '__main__':
    try:
        cmd()
    except rospy.ROSInterruptException:
        pass