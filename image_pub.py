#! /usr/bin/env python

# ============ Importing necessary libraries ===============

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

# ============ Naming the Node ==============================

rospy.init_node('input_from_camera' , anonymous=False)

# =========== Currently taking the input as a file ==========

img = cv2.imread(0) 
# img = cv2.imread("gate1.png")
#cv2.imshow('frame_pub', img)
#cv2.waitKey(1000)

# =========== Creating an object of class bridge ============

bridge = CvBridge()

# =========== generating a message which could be transmitted ========

img_msg = bridge.cv2_to_imgmsg(img, encoding="passthrough")

# ========== publishing the message to camera_input topic ===============================

pub = rospy.Publisher('camera_input', Image, queue_size=10)
rate= rospy.Rate(0.1)

while not rospy.is_shutdown():
    pub.publish(img_msg)
    rate.sleep()
