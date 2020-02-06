"""
Simple way to control each leg through a ui
Author: Patrick Gmerek
v2: Brian Henson

Not sure where the extra invisible buttons come from, or what they do, or the big black rectangle... but at least
the sliders are working as intended. Looks pretty messy, not at all like the screenshot in the docs, but it works.

When "Enable"=1, changes to the tip/mid/rot sliders are immediately written to the servos. When "enable"=0, its not.
Ideally, "Enable" would be a checkbox-type thing but CV only supports sliders ^_^

Select leg/group with the top slider: 0-5 = legs, 6=all, 7="left tri", 8="right tri"
When changing to a single leg, it reads the Leg's servos' current positions and sets the sliders to reflect it.
It also sets Enable to "on" because the sliders will already equal the pose its holding.
When changing to a group, it reads the relevant Leg's servos' current positions and sets the sliders to their
average position. But it sets Enable to "off" because this is not going to be the actual pose of any of the legs.

This uses the non-threading direct-set functions of the Leg objects.
"""
import sys
sys.path.append("../project_files/robot_drivers/")

import cv2 as cv
import time


# these must be globally available
all_legs = None
slider_names = ["Which Leg/Group", "Enable", 
				"Tip Joint", "Mid Joint", "Rot Joint"]
# first is init, second is max (min is always 0)
slider_limits = [[0, 8], [1, 1],
				 [0, 180], [90, 180], [90, 180]]
window_name = "Hexapod Leg Control"



def main():
	# init the legs and the window
	global all_legs
	all_legs = initialize_legs()
	cv.namedWindow(window_name)
	# create "which leg/group" slider with the changeleg() callback
	cv.createTrackbar(slider_names[0], window_name, slider_limits[0][0], slider_limits[0][1], changeleg)
	# create the other sliders with the dummy() callback
	for slider, limits in zip(slider_names[1:], slider_limits[1:]):
		cv.createTrackbar(slider, window_name, limits[0], limits[1], dummy)

	# initial read&set
	user_inputs = fetch_trackbar_pos(window_name, slider_names)
	all_legs[user_inputs[0]].set_leg_position(Leg_Position(user_inputs[2+ROT_SERVO], user_inputs[2+MID_SERVO], user_inputs[2+TIP_SERVO]))
	while True:
		previous_user_inputs = user_inputs
		user_inputs = fetch_trackbar_pos(window_name, slider_names)
		key = cv.waitKey(1) & 0xFF
		time.sleep(0.05)  # 20Hz refresh rate
		if key == ord("q"): # Quit if the user presses "q"
			break
		if not compare_lists(user_inputs, previous_user_inputs):
			# if the enable bar is set:
			if user_inputs[1] == 1:
				print("Values changed")
				newleg = Leg_Position(user_inputs[2+ROT_SERVO], user_inputs[2+MID_SERVO], user_inputs[2+TIP_SERVO])
				if user_inputs[0] == 6:
					# all legs at once
					for n in GROUP_ALL_LEGS:
						all_legs[n].set_leg_position(newleg)
				elif user_inputs[0] == 7:
					# left tri
					for n in GROUP_LEFT_TRI:
						all_legs[n].set_leg_position(newleg)
				elif user_inputs[0] == 8:
					# right tri
					for n in GROUP_RIGHT_TRI:
						all_legs[n].set_leg_position(newleg)
				else:
					all_legs[user_inputs[0]].set_leg_position(newleg)
					
	# only hit this after "q" pressed
	cv.destroyAllWindows()


def compare_lists(list1, list2):
	if not len(list1) == len(list1):
		return -1
	for i in range(0, len(list1)):
		if not list1[i] == list2[i]:
			return 0
	return 1


def fetch_trackbar_pos(mwindow_name, mslider_names):
	r = []
	for name in mslider_names:
		r.append(cv.getTrackbarPos(name, mwindow_name))
	return r

def changeleg(x):
	print("change leg")
	# function called when leg is changed
	if x == 6:
		# if x == 6, then selecting "all legs"...
		legs_list = GROUP_ALL_LEGS
		# set the "enable" trackbar to 0
		cv.setTrackbarPos(slider_names[1], window_name, 0)
	elif x == 7:
		legs_list = GROUP_LEFT_TRI
		cv.setTrackbarPos(slider_names[1], window_name, 0)
	elif x == 8:
		legs_list = GROUP_RIGHT_TRI
		cv.setTrackbarPos(slider_names[1], window_name, 0)
	else:
		legs_list = [x]
		# set the "enable" trackbar to 1 if selecting a single leg
		cv.setTrackbarPos(slider_names[1], window_name, 1)
		# lookup current angle values of leg and set trackbars to reflect that
		
	# given a leg or legs, calculate the average and set the sliders to that
	total = [0.0, 0.0, 0.0]
	# accumulate
	for n in legs_list:
		curr = all_legs[n].curr_servo_angle
		for s in GROUP_ALL_SERVOS:
			total[s] += curr[s]
	# divide and also set
	for s in GROUP_ALL_SERVOS:
		total[s] /= len(legs_list)
		total[s] = round(total[s])
		cv.setTrackbarPos(slider_names[s+2], window_name, int(total[s]))


def dummy(x):
	return


def initialize_legs():
	pwm_bot = pw.Pwm_Wrapper(PWM_ADDR_BOTTOM, PWM_FREQ)
	rf = hwd.Leg(pwm_bot, PWM_CHANNEL_ARRAY[LEG_RF], LEG_RF) #0
	rm = hwd.Leg(pwm_bot, PWM_CHANNEL_ARRAY[LEG_RM], LEG_RM) #1
	rb = hwd.Leg(pwm_bot, PWM_CHANNEL_ARRAY[LEG_RB], LEG_RB) #2
	larm = hwd.Leg(pwm_bot, PWM_CHANNEL_ARRAY[ARM_L], ARM_L) #6
	rot = hwd.Rotator(pwm_bot, PWM_CHANNEL_ARRAY[WAIST], WAIST) #8

	pwm_top = pw.Pwm_Wrapper(PWM_ADDR_TOP, PWM_FREQ)
	lb = hwd.Leg(pwm_top, PWM_CHANNEL_ARRAY[LEG_LB], LEG_LB) #3
	lm = hwd.Leg(pwm_top, PWM_CHANNEL_ARRAY[LEG_LM], LEG_LM) #4
	lf = hwd.Leg(pwm_top, PWM_CHANNEL_ARRAY[LEG_LF], LEG_LF) #5
	rarm = hwd.Leg(pwm_top, PWM_CHANNEL_ARRAY[ARM_R], ARM_R) #7
	
	return [rf, rm, rb, lb, lm, lf]


if __name__ == '__main__':
	main()
