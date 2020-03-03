"""
This module holds the global pwm variable and any utility functions

Authors:
    Brett Creeley
    Matty Baba Allos
"""
import threading


# these are the actual PWM object(s) and where they are instantiated... not very happy about this but w/e
# init with correct addresses
try:
    from Adafruit_PWM_Servo_Driver import PWM
    pwm_shield_0 = PWM(0x4A) # 0-15
    pwm_shield_1 = PWM(0x4C) # 16-31, currently no headers tho
    pwm_shield_2 = PWM(0x4B) # 32-47
    # assemble as list
    pwm_shields = [pwm_shield_0, pwm_shield_1, pwm_shield_2]
    # for each shield,
    for p in pwm_shields:
        # set correct frequency
        p.setPWMFreq(50)
        # wipe all values from past runs
        p.setAllPWM(0,0)
except ModuleNotFoundError:
    pwm_shields = [None, None, None]


# lock object to ensure only 1 thread touches the hats at a given instant
# i don't know exactly how long the "set pwm" operation takes, its probably instant, but there should
# be zero downside and only upside to making it thread-safe, since it is shared hardware.
_i2c_lock = threading.Lock()

# one function to set the PWM for any channel of any hat
def set_pwm(shield_id, channel_id, pulse_off):
    """ Set the pwm for the channel_id specified """
    
    # sanity-check values to be extra safe
    if not (0 <= pulse_off <= 4095):
        raise ValueError("PWM: invalid pulse_off %d" % pulse_off)
    # if not ((0 <= pulse_on <= 4096) and (0 <= pulse_off <= 4096) and (pulse_on <= pulse_off)):
    #     raise ValueError("PWM: invalid pulse_on %d pulse_off %d" % (pulse_on, pulse_off))

    # Don't allow invalid channel values
    if not (0 <= channel_id <= 15):
        raise ValueError("PWM: invalid channel_id %d" % channel_id)
    
    # Don't allow invalid shield values
    if not (0 <= shield_id < len(pwm_shields)):
        raise ValueError("PWM: invalid shield_id %d" % shield_id)

    # take the lock, set pwm, and release the lock
    with _i2c_lock:
        if pwm_shields[shield_id] is None:
            return pulse_off
        # actually do it, call .setPWM on the pwm object
        pwm_shields[shield_id].setPWM(channel_id, 0, pulse_off)
    return pulse_off


