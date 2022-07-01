#! /usr/bin/env python
from numpy import float32
import rospy
#from dynamic_reconfigure.server import Server
#from auv_codes2.cfg import thrusterConfig
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from std_msgs.msg import String

def accy_callback(msg):
	global acc_y
	acc_y = msg.data

def accx_callback(msg):
	global acc_x
	acc_x = msg.data

def gyrox_callback(msg):
	global gyro_x
	gyro_x = msg.data

def gyroy_callback(msg):
	global gyro_y
	gyro_y = msg.data

def gyroz_callback(msg):
	global gyro_z
	gyro_z = msg.data

def gyro_angle_x_callback(msg):
	global gyro_angle_x
	gyro_angle_x = msg.data

def gyro_angle_y_callback(msg):
	global gyro_angle_y
	gyro_angle_y = msg.data

def gyro_angle_z_callback(msg):
	global gyro_angle_z
	gyro_angle_z = msg.data

def acc_angle_x_callback(msg):
	global acc_angle_x
	acc_angle_x = msg.data

def acc_angle_y_callback(msg):
	global acc_angle_y
	acc_angle_y = msg.data

def accz_callback(msg):
	global acc_z
	acc_z = msg.data

def yaw_callback(msg):
	global yaw
	yaw = msg.data

def roll_callback(msg):
	global roll
	roll = msg.data

def pitch_callback(msg):
	global pitch
	pitch = msg.data



def dist_r_callback(msg):
	global distance_r
	distance_r = msg.data

def dist_l_callback(msg):
	global distance_l
	distance_l = msg.data

def con_r_callback(msg):
	global confidence_r
	confidence_r = msg.data

def con_l_callback(msg):
	global confidence_l
	confidence_l = msg.data


if __name__ == "__main__":

	sub_yaw = rospy.Subscriber("angle_z", Float64, yaw_callback)
    sub_accx = rospy.Subscriber("accx", Float64, accx_callback)
    sub_accy = rospy.Subscriber("accy", Float64, accy_callback)
    sub_accz = rospy.Subscriber("accz", Float64, accz_callback)
    sub_roll = rospy.Subscriber("angle_x", Float64, roll_callback)
    sub_pitch = rospy.Subscriber("angle_y", Float64, pitch_callback)
    sub_gyrox = rospy.Subscriber("gyrox", Float64, gyrox_callback)
    sub_gyroy = rospy.Subscriber("gyroy", Float64, gyroy_callback)
    sub_gyroz = rospy.Subscriber("gyroz", Float64, gyroz_callback)
    sub_gyro_angle_x = rospy.Subscriber("gyro_angle_x", Float64, gyro_angle_x_callback)
    sub_gyro_angle_y = rospy.Subscriber("gyro_angle_y", Float64, gyro_angle_y_callback)
    sub_gyro_angle_z = rospy.Subscriber("gyro_angle_z", Float64, gyro_angle_z_callback)
    sub_acc_angle_x = rospy.Subscriber("acc_angle_x", Float64, acc_angle_x_callback)
    sub_acc_angle_y = rospy.Subscriber("acc_angle_y", Float64, acc_angle_y_callback)


    sub_dist_r = rospy.Subscriber("dist_r", Float64, dist_r_callback)
    sub_dist_l = rospy.Subscriber("dist_l", Float64, dist_l_callback)
    sub_con_r = rospy.Subscriber("con_r", Float64, con_r_callback)
    sub_con_l = rospy.Subscriber("con_l", Float64, con_l_callback)
