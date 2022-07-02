#! /usr/bin/env python
from numpy import float32
import rospy
#from dynamic_reconfigure.server import Server
#from auv_codes2.cfg import thrusterConfig
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from std_msgs.msg import String

yaw = 0
acc_y = 0
prev_err_y, int_err_y, prev_err_a, int_err_a = 0,0,0,0
KP_y, KD_y, KI_y = 0,0,0
KP_a, KD_a, KI_a = 0,0,0
err_y_thresh = 10 #degrees
err_a_thresh = 0
turning _factor = 0
MAXpwm_fr, MAXpwm_fl, MAXpwm_br, MAXpwm_bl, MAXpwm_mr, MAXpwm_ml = 1900, 1900, 1900, 1900, 1900, 1900
MINpwm_fr, MINpwm_fl, MINpwm_br, MINpwm_bl, MINpwm_mr, MINpwm_ml = 1100, 1100, 1100, 1100, 1100, 1100
pwm_fr, pwm_br, pwm_fl, pwm_bl, pwm_mr, pwm_ml = 1600, 1600, 1600, 1600, 1500, 1500 

#err_y is positive => robot is turned towards right, when we are looking from back

def straightLine_pid_imu():
	global prev_err_y, int_err_y, prev_err_a, int_err_a
	global pwm_fr, pwm_br, pwm_fl, pwm_bl, pwm_mr, pwm_ml

	err_y = yaw
	diff_err_y = err_y - prev_err_y
	int_err_y += err_y 
	
	correction_y = KP_y*abs(err_y) + KD_y*diff_err_y
	correction_y = (int_err_y*err_y > 0) ? correction_y + KI_y*int_err_y : correction_y - KI_y*int_err_y 
		
	err_a = acc_y
	diff_err_a = err_a - prev_err_a
	int_err_a += err_a 
	
	correction_a += KP_a*abs(err_a) + KD_a*diff_err_a
	correction_a = (int_err_a*err_a > 0) ? correction_a + KI_a*int_err_a : correction_a -KI_a*int_err_a

	if(err_y > err_y_thresh):
		pwm_fr += correction_y #front right should rotate in a way such that one component of it's force is towards left
		pwm_bl += correction_y - turning_factor #to go forward and at the same time turn slightly left

		pwm_fr = min(pwm_fr, MAXpwm_fr)
		pwm_fr = max(pwm_fr, MINpwm_fr)
		pwm_bl = min(pwm_bl, MAXpwm_bl)
		pwm_bl = max(pwm_bl, MINpwm_bl)
	
	elif(err_y < -err_y_thresh):
		pwm_fl += correction_y
		pwm_br += correction_y - turning_factor
		pwm_fl = min(pwm_fl, MAXpwm_fl)
		pwm_fl = max(pwm_fl, MINpwm_fl)
		pwm_br = min(pwm_br, MAXpwm_br)
		pwm_br = max(pwm_br, MINpwm_br)
	
	elif(abs(err_a) > err_a_thresh):
		if(err_a>0):
			pwm_fr += correction_a
			pwm_bl += correction_a

			pwm_fr = min(pwm_fr, MAXpwm_fr)
			pwm_fr = max(pwm_fr, MINpwm_fr)
			pwm_bl = min(pwm_bl, MAXpwm_bl)
			pwm_bl = max(pwm_bl, MINpwm_bl)
		else:
			pwm_fl += correction_a
			pwm_br += correction_a
			pwm_fl = min(pwm_fl, MAXpwm_fl)
			pwm_fl = max(pwm_fl, MINpwm_fl) 
			pwm_br = min(pwm_br, MAXpwm_br)
			pwm_br = max(pwm_br, MINpwm_br)
			
	pwm_msg = str(pwm_fr) + ' ' + str(pwm_fl) + ' ' + str(pwm_mr) + ' ' + str(pwm_ml) + ' ' + str(pwm_br) + ' ' + str(pwm_bl) + ' '; 
	pub.Publish(pwm_msg);
	return err_y, correction_y, err_a, correction_a

def moveForward:
	err_y, correction_y, err_a, correction_a = straightLine_pid_imu()
	
def accy_callback(msg):
	global acc_y
	acc_y = msg.data

def yaw_callback(msg):
	global yaw
	yaw = msg.data
	straightLine_pid_imu()

if __name__ == "__main__":
	
    rospy.init_node("surf_straight_pid", anonymous = False)
    q = 1
    
    sub_yaw = rospy.Subscriber("angle_z", Float64, yaw_callback)
    sub_accx = rospy.Subscriber("accx", Float64, accx_callback)
    sub_accy = rospy.Subscriber("accy", Float64, accy_callback)
	
    pub=rospy.Publisher("PWM_VALUE",String ,queue_size=q)
   
    rospy.spin()
  
