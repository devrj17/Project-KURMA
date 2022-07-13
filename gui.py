#! /usr/bin/env python

# from sympy import true
import rospy

from dynamic_reconfigure.server import Server
from auv_codes2.cfg import thrusterConfig
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import String

err=30
horizontalDistance=0


def turnRight():
    pwm_fr=1450
    pwm_fl=1550
    pwm_mr=1500
    pwm_ml=1500
    pwm_br=1550
    pwm_bl=1450

    pwm_value= str(pwm_fr) +' '+ str(pwm_fl) +' '+str(pwm_mr)+ ' '+ str(pwm_ml)+ ' '+ str(pwm_br)+' '+ str(pwm_bl)
    pub.publish(pwm_value) 
    rospy.loginfo("inside right")
    rospy.sleep(0.5)
    # cam()

def turnLeft() :
    pwm_fr=1550
    pwm_fl=1450
    pwm_mr=1500
    pwm_ml=1500
    pwm_br=1450
    pwm_bl=1550

    rospy.loginfo("inside left")
    pwm_value= str(pwm_fr) +' '+ str(pwm_fl) +' '+str(pwm_mr)+ ' '+ str(pwm_ml)+ ' '+ str(pwm_br)+' '+ str(pwm_bl)
    pub.publish(pwm_value) 
    rospy.sleep(0.5)
    # cam()

def straight() :
    pwm_fr=1550
    pwm_fl=1550
    pwm_mr=1500
    pwm_ml=1500
    pwm_br=1550
    pwm_bl=1550

    pwm_value= str(pwm_fr) +' '+ str(pwm_fl) +' '+str(pwm_mr)+ ' '+ str(pwm_ml)+ ' '+ str(pwm_br)+' '+ str(pwm_bl)
    pub.publish(pwm_value) 
    rospy.loginfo("inside straight")
    rospy.sleep(0.5)
    # cam()


def cam() :

    rospy.loginfo("inside cam")
    if horizontalDistance>-err and horizontalDistance<err :
        straight()
    elif horizontalDistance>err :
        turnLeft() 
    else :
        turnRight()


def callback_verify(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def callback_imu(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def callback_pressure(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def callback_distance(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard front and horizontal distance %s", data.data)
    dist=data.data
    distances=dist.split()
    front_distance=float(distances[0])
    horizontalDistance=float(distances[1])

    
    # rospy.loginfo(horizontalDistance)

def rotate_right(angle):
    pwm_fr = 1500
    pwm_fl = 1500
    pwm_mr = 1500
    pwm_ml = 1500
    pwm_br = 1500
    pwm_bl = 1500
    k= 0.1
    t= k*angle
    rospy.loginfo("inside rotate")
    rospy.sleep(1)

    pwm_value= str(pwm_fr) +' '+ str(pwm_fl) +' '+str(pwm_mr)+ ' '+ str(pwm_ml)+ ' '+ str(pwm_br)+' '+ str(pwm_bl)
    pub.publish(pwm_value) 
    
    # rospy.sleep(t)

    rospy.loginfo("after sleep")

    pwm_fr = 1500
    pwm_fl = 1500
    pwm_mr = 1500
    pwm_ml = 1500
    pwm_br = 1600
    pwm_bl = 1600

    pwm_value= str(pwm_fr) +' '+ str(pwm_fl) +' '+str(pwm_mr)+ ' '+ str(pwm_ml)+ ' '+ str(pwm_br)+' '+ str(pwm_bl)
    pub.publish(pwm_value) 

    rospy.sleep(t)

    rospy.loginfo("after sleep")

    pwm_fr = 1500
    pwm_fl = 1500
    pwm_mr = 1500
    pwm_ml = 1500
    pwm_br = 1500
    pwm_bl = 1500

    pwm_value= str(pwm_fr) +' '+ str(pwm_fl) +' '+str(pwm_mr)+ ' '+ str(pwm_ml)+ ' '+ str(pwm_br)+' '+ str(pwm_bl)
    pub.publish(pwm_value) 



def callback_conf(config, level):
    rospy.loginfo("""Reconfigure Request: {servoFrontRight}, {servoFrontLeft},{servoMiddleRight}, {servoMiddleLeft}, {servoBackLeft}, {servoBackRight},{mode_thrusters}""".format(**config))
   
    




    
    mode   = config.mode_thrusters

    if (mode==0):
            #stop all thrusters
        print('mode is ', mode)
        pwm_fr = 1500
        pwm_fl = 1500
        pwm_mr = 1500
        pwm_ml = 1500
        pwm_br = 1500
        pwm_bl = 1500

        pwm_value= str(pwm_fr) +' '+ str(pwm_fl) +' '+str(pwm_mr)+ ' '+ str(pwm_ml)+ ' '+ str(pwm_br)+' '+ str(pwm_bl)
        pub.publish(pwm_value)

    elif  (mode==1):
               #all thrusters value to 1600 to check every thruster is working
        print('mode is ', mode)   
        pwm_fr = 1600
        pwm_fl = 1600
        pwm_mr = 1600
        pwm_ml = 1600
        pwm_br = 1600
        pwm_bl = 1600

        pwm_value= str(pwm_fr) +' '+ str(pwm_fl) +' '+str(pwm_mr)+ ' '+ str(pwm_ml)+ ' '+ str(pwm_br)+' '+ str(pwm_bl)
        pub.publish(pwm_value)
    elif  (mode==2):  
        
              # value receieved from reconfigure
        print('mode is ', mode)
        pwm_fr = config.servoFrontRight
        pwm_fl = config.servoFrontLeft
        pwm_mr = config.servoMiddleRight
        pwm_ml = config.servoMiddleLeft
        pwm_br = config.servoBackRight
        pwm_bl = config.servoBackLeft
        pwm_value= str(pwm_fr) +' '+ str(pwm_fl) +' '+str(pwm_mr)+ ' '+ str(pwm_ml)+ ' '+ str(pwm_br)+' '+ str(pwm_bl)
        pub.publish(pwm_value)

    elif  (mode==3): 
        print('mode is ', mode)
               # value from camera 
        rotate_right(90)
    elif  (mode==4):         # value from imu sensor 

        print('mode is ', mode)
        # while(mode==4):
        #     cam()
        
    elif  (mode==5):        # value from camera 

        print('mode is ', mode)
        pwm_fr = config.servoFrontRight
        pwm_fl = config.servoFrontLeft
        pwm_mr = config.servoMiddleRight
        pwm_ml = config.servoMiddleLeft
        pwm_br = config.servoBackRight
        pwm_bl = config.servoBackLeft


    print("Out of if else")
    
           
    # pub7.publish(stop)


    return config

if __name__ == "__main__":
    rospy.init_node("thruster_value", anonymous = False)
    q = 1
    pub=rospy.Publisher("PWM_VALUE",String ,queue_size=q)
    rospy.Subscriber("Verify", String, callback_verify)
    # rospy.Subscriber("imu_data", String, callback_imu)
    # rospy.Subscriber("pressure_data", String, callback_pressure)
    rospy.Subscriber("object_distance", String, callback_distance)
    #rotate_right(90)
    


    srv = Server(thrusterConfig, callback_conf)
    rospy.spin()
