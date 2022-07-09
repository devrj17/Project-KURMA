#! /usr/bin/env python
from numpy import float32
import rospy
#from dynamic_reconfigure.server import Server
#from auv_codes2.cfg import thrusterConfig
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from std_msgs.msg import String

import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Create figure for plotting

xs = []
ys1 = []
ys2 = []

acc_y = 0
acc_x = 0
gyro_x = 0
gyro_y = 0
gyro_z = 0
gyro_angle_x = 0
gyro_angle_y = 0
gyro_angle_z = 0
acc_angle_x = 0
acc_angle_y = 0
acc_z = 0
yaw = 0
roll = 0
pitch = 0
distance_r = 0
distance_l = 0
confidence_r = 0
confidence_l = 0


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



# This function is called periodically from FuncAnimation
def animate(i):

    rospy.Subscriber("angle_z", Float64, yaw_callback)
    rospy.Subscriber("accx", Float64, accx_callback)
    rospy.Subscriber("accy", Float64, accy_callback)
    rospy.Subscriber("accz", Float64, accz_callback)
    rospy.Subscriber("angle_x", Float64, roll_callback)
    rospy.Subscriber("angle_y", Float64, pitch_callback)
    rospy.Subscriber("gyrox", Float64, gyrox_callback)
    rospy.Subscriber("gyroy", Float64, gyroy_callback)
    rospy.Subscriber("gyroz", Float64, gyroz_callback)
    rospy.Subscriber("gyro_angle_x", Float64, gyro_angle_x_callback)
    rospy.Subscriber("gyro_angle_y", Float64, gyro_angle_y_callback)
    rospy.Subscriber("gyro_angle_z", Float64, gyro_angle_z_callback)
    rospy.Subscriber("acc_angle_x", Float64, acc_angle_x_callback)
    rospy.Subscriber("acc_angle_y", Float64, acc_angle_y_callback)


    rospy.Subscriber("dist_r", Float64, dist_r_callback)
    rospy.Subscriber("dist_l", Float64, dist_l_callback)
    rospy.Subscriber("con_r", Float64, con_r_callback)
    rospy.Subscriber("con_l", Float64, con_l_callback)


    # Add x and y to lists
    xs.append(dt.datetime.now().strftime('%H:%M:%S.%f'))
    ys1.append(acc_y)
    ys2.append(acc_x)
    # ys.append(gyro_x)
    # ys.append(gyro_y)
    # ys.append(gyro_z)
    # ys.append(gyro_angle_x)
    # ys.append(gyro_angle_y)
    # ys.append(gyro_angle_z)
    # ys.append(acc_angle_x)
    # ys.append(acc_angle_y)
    # ys.append(acc_z)
    # ys.append(yaw)
    # ys.append(roll)
    # ys.append(pitch)
    # ys.append(distance_r)
    # ys.append(distance_l)
    # ys.append(confidence_r)
    # ys.append(confidence_l)
    # ys.append()
  
    # Limit x and y lists to 20 items
    # xs = xs[-20:]
    # ys = ys[-20:]

    plt.cla()

    plt.plot(xs, ys1, label='Channel 1')
    plt.plot(xs, ys2, label='Channel 1')
    
    plt.tight_layout()

if __name__ == "__main__":

    ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys), interval=1000)
    plt.tight_layout()
    plt.show()
