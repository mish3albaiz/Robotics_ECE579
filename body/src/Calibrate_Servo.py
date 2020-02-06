"""
Authors:
    Brett Creeley
    Matty Baba Allos
"""
#NOTE: This file is used as sandbox file and to get max and min values for servos


#!/bin/python

# Returns servo pulse how the library wants it based
# on the input being a datasheet value in microseconds
def determineServoPulse(pulse_us_ds, hz):
    micro_per_sec = 1000000
    period = micro_per_sec / hz
    time_per_tick = period / 4096 # 12-bit resolution
    return pulse_us_ds / time_per_tick


#!/bin/python
import time
from Adafruit_PWM_Servo_Driver import PWM
from Servo import Servo


# Left/Right Torso - HS-805BB
servoMin = determineServoPulse(800, 50)
servoMax = determineServoPulse(2100, 50)
servoMid = determineServoPulse(1500, 50)
servo135 = determineServoPulse(1900, 50)

pwm = PWM(0x40)
pwm.setPWMFreq(50)

# servoMin = servoMin
# servoMax = servoMax

# print "ServoMin = ", servoMin
# print "ServoMax = ", servoMax

def head_up():
     print("head up")
     pwm.setPWM(0, 0, servoMax)
     pwm.setPWM(15, 0, servoMax)

def head_down():
     print("head down")
     pwm.setPWM(15, 0, servoMin)

     pwm.setPWM(0, 0, servoMin)

def off():
    """Move head up"""
    pwm.setPWM(15, 0, 0)
    pwm.setPWM(0, 0, 0)



head_up()
time.sleep(1)




# def head_down():
#     '''Move head down'''
#     pwm.setPWM(2, 0, servoMin)

# def head_mid_y():
#     '''Move head to the middle (y-axis)'''
#     pwm.setPWM(2, 0, servoMid)

# def head_left():
#     '''Move head to the left'''
#     pwm.setPWM(3, 0, servoMax)

# def head_right():
#     '''Move head to the right'''
#     pwm.setPWM(3, 0, servoMin)

# def head_mid_x():
#     '''Move head to the middle (x-axis)'''
#     pwm.setPWM(3, 0, servoMid-25)

# def lean_left():
#     '''Lean to the left'''
#     pwm.setPWM(0, 0, servoMax)
#     pwm.setPWM(1, 0, servoMax)

# def lean_right():
#     '''Lean to the right'''
#     pwm.setPWM(0, 0, servoMin)
#     pwm.setPWM(1, 0, servoMin)

# def lean_center():
#     '''Center the torso'''
#     pwm.setPWM(0, 0, servoMid)
#     pwm.setPWM(1, 0, servoMid)

# def initialize():
#     '''Initialize to all centered locations'''
#     head_mid_x()
#     head_mid_y()
#     lean_center()
