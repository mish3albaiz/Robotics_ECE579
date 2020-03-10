#!/usr/bin/env python

# this testing file is to directly set the angle/PWM values of a single servo
# use extreme caution, this can easily damage the servos or drain the battery if specifying a PWM
# outside the range the servos can actually reach

import sys
from os.path import join, dirname
whereami = dirname(__file__)
scriptsdir = join(whereami, "../scripts/")
sys.path.append(scriptsdir)
import Inmoov
from Servo import linear_map

# this should set all servos to "off" on bootup
bot = Inmoov.Inmoov()

this_python_version = sys.version_info.major
# define which input capture function to use, depending on python version
if this_python_version == 2:
    # python 2
    my_input = raw_input
if this_python_version == 3:
    # python 3
    my_input = input


###############################################################

def main():
    print("")
    print("This file is for directly setting the PWM values of given servos to determine their active range.")
    print("This ignores the PWM limits listed in the drivers!!!")
    print("This means it is easily possible to damage the servos so be extremely careful! This is intended for determining new limits of the servos.")
    print("Abort with ctrl+C to turn off all servos and end the program.")
    print("USE EXTREME CAUTION")
    print("")
    
    # once non-numeric value given, go back one level
    print("Enter a non-numeric input to go up/back a level")
    
    try:
        while True:
            # select which servo by name
            r = my_input("Enter servo name:")
            if r == "":
                break
            s = bot.find_servo_by_name(r)
            # if servo is not found, then prompt again
            if s is None:
                continue
            print(s)
            # if servo is found, then print its min/max/current
            print("MIN=%d, MAX=%d, CURR=%d" % (s.min_pulse, s.max_pulse, s.curr_pwm))
            
            while True:
                # prompt for actual value
                r = my_input("  PWM:")
                P = -1
                try:
                    P = int(r)
                except ValueError:
                    # if given non-numeric input, go up a level
                    break
                
                # actually set the PWM, function actually wants degrees so first need to convert
                # here we map pwm -> degrees, inside the function it maps degrees -> pwm
                # floating-point math so there is no loss
                deg = linear_map(P,s.min_pulse,s.max_pulse,s.min_angle,s.max_angle)
                s.do_set_angle(deg)
            
    finally:
        print("exiting")
        bot.off()


if __name__ == '__main__':
    main()

