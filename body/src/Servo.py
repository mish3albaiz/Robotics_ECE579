#!/bin/python

"""
Authors:
    Brett Creeley
    Matty Baba Allos
"""
from Config import set_pwm
import warnings
import threading
import frame_thread as ft


# prints an assload of debug statements for the servo with the specified ID
# there is no servo with id -1 so that will turn off debugging
DEBUG_SERVO_ID = -1



# basic clamp
def clamp(value, lower, upper):
    return lower if value < lower else upper if value > upper else value
# clamp where you dont know the relative order of a and b
def bidirectional_clamp(val, a, b):
    return clamp(val, a, b) if a < b else clamp(val, b, a)

# map a value along one range onto another range
def linear_map(x_in_val, x1, x2, y1, y2):
    m = (y2 - y1) / (x2 - x1)
    b = y2 - m * x2
    return x_in_val * m + b



class Servo(object):
    """
    Servo stores the following values:
        channel: The channel the Servo is connected to
        min_pulse: The minimum pulse the Servo allows
        max_pulse: The maximum pulse the Servo allows
        min_degree: The minimum degree the Servo can rotate to
        max_degree: The maximum degree the Servo can rotate to
        default_angle: The rest(initial)position of the servo.
        name: The name of the servo
    """

    def __init__(self,
                 servo_id,
                 min_pulse,
                 max_pulse,
                 min_degree=None,
                 max_degree=None,
                 default_angle=None,
                 name="servo",
                 disabled=False
                 ):
        
        self.disabled = disabled
        if self.disabled:
            servo_id = 47
            print("Warning: servo '%s' is disabled" % name)
        self.servo_id = servo_id
        # servo IDs start from 0, and because there are only 16 channels on a hat, the actual channel is ID mod 16.
        # also, the hat number is ID // 16 (round-down division).
        # Example: ID=20, 20 % 16 = 4, 20 // 16 = 1. Channel 4, hat 1.
        self.channel = int(servo_id % 16)
        self.shield_id = int(servo_id // 16)
        self.min_pulse = min_pulse
        self.max_pulse = max_pulse
        self.min_degree = min_degree
        self.max_degree = max_degree
        self.default_angle = default_angle
        self.name = name
        # sanity checking, in case of typos in the config json
        if not self.disabled:
            # currently we do not prevent either degrees or pwm from being upside-down, this might change in future
            assert 1 <= max_pulse <= 4096
            assert 1 <= min_pulse <= 4096
            assert 0 <= servo_id <= 50
            assert -360 <= min_degree <= 360
            assert -360 <= max_degree <= 360
            assert min(min_degree, max_degree) <= default_angle <= max(min_degree, max_degree)
        
        # running/idle flags: normal Events can only wait for a rising edge, if I want to wait for a falling edge, i need to
        # set up a complementary system like this. also they're mostly being used as flags, not as "events", but whatever.
        self.running_flag = threading.Event()
        self.idle_flag = threading.Event()
        self.idle_flag.set()
        # i want setting one/clearing other to be an indivisible atomic operation so it should have a lock object just in case
        self._state_flag_lock = threading.Lock()
        # the list of frames that the leg thread is consuming as the leg object is adding onto
        self.frame_queue = []
        # locking object to ensure no collisions happen around the frame queue
        self._frame_queue_lock = threading.Lock()
        # these variables track the state of the servo at any given instant
        self.curr_angle = 0.0
        self.curr_pwm = 0
        self.curr_on = False
        # locking object to ensure no collisions happen around self.curr_angle/self.curr_pwm/self.curr_on
        # might not be necessary but couldn't hurt, technically both the servo thread and servo object can write into them
        self._curr_state_lock = threading.Lock()

        # create and launch the thread for this servo
        # note: this MUST be daemon type because the thread is designed to run forever...
        # the only way to stop it is by stopping its parent, which means it must be a daemon!
        # it will be able to access all of this servos's other member variables and functions
        self.framethread_name = "framethread_" + self.name
        self.framethread = threading.Thread(name=self.framethread_name, daemon=True,
                                            target=ft.Frame_Thread_Func, args=(self, DEBUG_SERVO_ID))
        

        # start the thread, this should be the 2nd last operation of __init__
        self.framethread.start()
        # the servo begins as "off" until explicitly told to initialize it
        self.off()
        
    def __str__(self) -> str:
        # self.name, self.id, self.channel, self.shield_id, self.min_pulse, self.max_pulse, self.min_degree, self.max_degree, self.default_angle, self.disabled
        # curr_angle, curr_pwm, curr_on
        s = "name={}, id={}, channel={}, shield_id={}, min_pulse={}, max_pulse={}, min_degree={}, max_degree={}, default_angle={}, disabled={}\ncurr_angle={}, curr_pwm={}, curr_on={}"
        return s.format(self.name, self.id, self.channel, self.shield_id, self.min_pulse, self.max_pulse,
                       self.min_degree, self.max_degree, self.default_angle, self.disabled,
                        self.curr_angle, self.curr_pwm, self.curr_on)



    def rotate(self, degree: float):
        # todo: rename
        """ Rotate to the specified degrees """
        # non-threading method of controlling the servo
        # perform safety clamp before passing to do_set_angle
        degree_safe = self.degrees_clamp(degree)
        # calling the non-threading control function should cancel any pending threading events
        self.abort()
        try:
            self.do_set_angle(degree_safe)
        except ValueError as exception:
            print(exception)
            print("Could not rotate {} to {} degree").format(self.name, degree)


    # safety clamp (in angle space)
    # interpolate (in angle space)
    # adds frames to the frame queue (with lock)
    # sets the "running" flag unconditionally (note: no harm in setting an already set flag)
    # * thread will jump in with "do_set_servo_angle" when it is the correct time
    def rotate_thread(self, degree: float, durr: float):
        # safety clamp
        dest = self.degrees_clamp(degree)
        
        # if there is a queued interpolation frame, interpolate from the final frame in the queue to the desired pose.
        # otherwise, interpolate from current position.
        curr = None
        with self._frame_queue_lock:
            if len(self.frame_queue) > 0:
                curr = self.frame_queue[-1][0]
        if curr is None:  # "else" but outside of the lock block
            # floats are always copied, not referenced
            curr = self.curr_angle

        if self.id == DEBUG_SERVO_ID:
            print("servo_%s: interp from deg %d to %d over %f" % (self.name, curr, dest, durr))
        
        # run interpolation
        interp_list = ft.interpolate(dest, curr, durr)
    
        # add new frames onto the END of the frame queue (with lock)
        with self._frame_queue_lock:
            # concatenate two lists with +
            self.frame_queue = self.frame_queue + interp_list
            if self.id == DEBUG_SERVO_ID:
                print("servo_%s: add %d frames, new length %d" % (self.name, len(interp_list), len(self.frame_queue)))
    
        with self._state_flag_lock:
            # clear "sleeping" event, does not trigger anything (note: clear before set)
            self.idle_flag.clear()
            # set the "running" event, this will trigger the thread to begin consuming frames
            self.running_flag.set()


    def do_set_angle(self, degree: float):
        # take angle after clamp, already known to be safe
        # called by both threading and non-threading approaches
        # convert to pwm
        # with lock, set pwm
        pulse = self.degrees_to_pulse(degree)

        with self._curr_state_lock:
            self.curr_on = True
            self.curr_angle = degree
            self.curr_pwm = pulse
            set_pwm(self.shield_id, self.channel, pulse)

    def initialize(self):
        """ Move servo to defult position """
        print("init name = {}, default_angle = {}".format(self.name, self.default_angle))
        self.rotate(self.default_angle)

    def off(self):
        """ Rotate to the specified degrees """
        # TODO: i'm not conviced setting this to 0 is a good idea, ever... does it drive to zero-position or cut power?
        
        # abort so it doesn't wake up after turning off until i explicitly tell it to wake up
        self.abort()
        try:
            with self._curr_state_lock:
                self.curr_on = False
                self.curr_angle = None
                self.curr_pwm = 0
                set_pwm(self.shield_id, self.channel, 0)
        except ValueError as exception:
            print(exception)
            print("Could not turn off {}").format(self.name)


    # clear the frame queue to stop any currently-pending movements.
    def abort(self):
        with self._frame_queue_lock:
            self.frame_queue = []





    def degrees_clamp(self, degree: float) -> float:
        # clamp for safety
        degree_safe = bidirectional_clamp(degree, self.min_degree, self.max_degree)
        # warn if clamping actually occurred
        if degree != degree_safe:
            warnings.warn("Degree {} is out of range, clamping to safe value {}".format(degree, degree_safe))
        return degree_safe
    
    def degrees_to_pulse(self, degree: float) -> int:
        """ Map degree input value to a pulse length output value """
        # perform actual mapping
        pulse = int(linear_map(degree, self.min_degree, self.max_degree, self.min_pulse, self.max_pulse))
        return pulse

# removed setters & getters cuz parameters are not redefined after initialization, therefore they are useless