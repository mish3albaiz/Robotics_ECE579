#!/bin/python
"""
This module holds the global pwm variable and any utility functions

Authors:
    Brett Creeley
    Matty Baba Allos
"""
from Adafruit_PWM_Servo_Driver import PWM
import threading


# these are the actual PWM object(s) and where they are instantiated... not very happy about this but w/e
pwm_shield_0 = PWM(0x4A)
pwm_shield_1 = PWM(0x4C)
pwm_shield_2 = PWM(0x4B)
pwm_shield_0.setPWMFreq(50)
pwm_shield_1.setPWMFreq(50)
pwm_shield_2.setPWMFreq(50)

pwm_shields = [pwm_shield_0, pwm_shield_1, pwm_shield_2]

# lock object to ensure only 1 thread touches the hats at a given instant
# i don't know exactly how long the "set pwm" operation takes, its probably instant, but there should
# be zero downside and only upside to making it thread-safe, since it is shared hardware.
_pwm_lock = threading.Lock()

# one function to set the PWM for any channel of any hat
def set_pwm(shield_id: int, channel_id: int, pulse_off: int):
    """ Set the pwm for the channel_id specified """
    # note that pulse_on should ALWAYS be 0
    
    
    # sanity-check values to be extra safe
    if not (0 <= pulse_off <= 4096):
        raise ValueError("PWM: invalid pulse_off %d" % pulse_off)
    # if not ((0 <= pulse_on <= 4096) and (0 <= pulse_off <= 4096) and (pulse_on <= pulse_off)):
    #     raise ValueError("PWM: invalid pulse_on %d pulse_off %d" % (pulse_on, pulse_off))

    """ Don't allow invalid channel values """
    if not (0 <= channel_id <= 15):
        raise ValueError("PWM: invalid channel_id %d" % channel_id)

    # take the lock, set pwm, and release the lock
    with _pwm_lock:
        try:
            # actually do it, call .setPWM on the pwm object
            pwm_shields[shield_id].setPWM(channel_id, 0, pulse_off)
        except IndexError:
            # turn index error into value error because i can
            raise ValueError("PWM: invalid shield_id %d" % shield_id)
    return pulse_off


