def depth_pid():
	global prev_err_d, int_err_d, prev_err_r, int_err_r

	err_y = 0.7 - depth
	diff_err_d = err_d - prev_err_d
	int_err_d += err_d 

correction_d = KP_d*abs(err_d) + KD_d*diff_err_d
	correction_d = (int_err_d*err_d > 0) ? correction_d + KI_y*int_err_y : correction_d -KI_d*int_err_d

	err_r = roll
	diff_err_r = err_r - prev_err_r
	int_err_r += err_r 

correction_r = KP_r*abs(err_r) + KD_r*diff_err_r
	correction_r = (int_err_r*err_r > 0) ? correction_r + KI_r*int_err_r : correction_r -KI_r*int_err_r

if(error_d > err_d_thresh):
pwm_mr += correction_d
	pwm_ml += correction_d

	pwm_mr = min(pwm_fr, MAXpwm_mr)
pwm_mr = max(pwm_fr, MINpwm_mr)
pwm_ml = min(pwm_bl, MAXpwm_ml)
pwm_ml = max(pwm_bl, MINpwm_ml)
elif(err_d < -err_d_thresh):
	pwm_mr -= correction_d
	pwm_ml -= correction_d

	pwm_mr = min(pwm_fr, MAXpwm_mr)
pwm_mr = max(pwm_fr, MINpwm_mr)
pwm_ml = min(pwm_bl, MAXpwm_ml)
pwm_ml = max(pwm_bl, MINpwm_ml)

	if(error_r > err_r_thresh):
pwm_mr -= correction_d
	pwm_ml += correction_d

	pwm_mr = min(pwm_fr, MAXpwm_mr)
pwm_mr = max(pwm_fr, MINpwm_mr)
pwm_ml = min(pwm_bl, MAXpwm_ml)
pwm_ml = max(pwm_bl, MINpwm_ml)
elif(err_r < -err_r_thresh):
	pwm_mr += correction_d
	pwm_ml -= correction_d

	pwm_mr = min(pwm_fr, MAXpwm_mr)
pwm_mr = max(pwm_fr, MINpwm_mr)
pwm_ml = min(pwm_bl, MAXpwm_ml)
pwm_ml = max(pwm_bl, MINpwm_ml)


def straightLine_pid_imu():
	global prev_err_y, int_err_y, prev_err_a, int_err_a

	err_y = yaw
	diff_err_y = err_y - prev_err_y
	int_err_y += err_y 
	
	correction_y = KP_y*abs(err_y) + KD_y*diff_err_y
	correction_y = (int_err_y*err_y > 0) ? correction_y + KI_y*int_err_y : correction_y - KI_y*int_err_y 

err_a = acceleration_ydirn
	diff_err_a = err_a - prev_err_a
	int_err_a += err_a 
	
	correction_a += KP_a*abs(err_a) + KD_a*diff_err_a
	correction_a = (int_err_a*err_a > 0) ? correction_a + KI_a*int_err_a : correction_a -KI_a*int_err_a

if(err_y > err_y_thresh):
	pwm_fr += correction_y
	pwm_bl += correction_y - turning_factor

	pwm_fr = min(pwm_fr, MAXpwm_fr)
pwm_fr = max(pwm_fr, MINpwm_fr)
pwm_bl = min(pwm_bl, MAXpwm_bl)
pwm_bl = max(pwm_bl, MINpwm_bl)

pwm_fl, pwm_br = 1500,1500
elif(err_y < -err_y_thresh):
	pwm_fl += correction_y
	pwm_br += correction_y - turning_factor
	pwm_fl = min(pwm_fl, MAXpwm_fl)
pwm_fl = max(pwm_fl, MINpwm_fl)
pwm_br = min(pwm_br, MAXpwm_br)
pwm_br = max(pwm_br, MINpwm_br)

 	pwm_fr, pwm_bl = 1500,1500
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

 	pwm_fr, pwm_bl = 1500,1500

return err_y, correction_y, err_a, correction_a

def straightLine_pid_gate():
	err_y, correction_y, err_a, correction_a = straightLine_pid_imu()
	
	if(abs(err_y) < err_y_thresh):
		if (cx>360):
			
		elif(cx<280):


def moveForward:
depth_pid()
if(gate_visible):
straightLine_pid_gate()
else:
err_y, correction_y, err_a, correction_a = straightLine_pid_imu()

def depth_callback():
	moveForward()

  
