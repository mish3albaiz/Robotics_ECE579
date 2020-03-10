
"""
Authors:
    Brett Creeley
    Matty Baba Allos
"""
from Pwm_Interface import set_pwm
import threading
import Frame_Thread as ft


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
    x_in_val, x1, x2, y1, y2 = float(x_in_val), float(x1), float(x2), float(y1), float(y2)
    m = (y2 - y1) / (x2 - x1)
    b = y2 - m * x2
    return x_in_val * m + b



class Servo(object):
    """
    Servo stores the following values:
        channel_id: The channel the Servo is connected to
        min_pulse: The minimum pulse the Servo allows
        max_pulse: The maximum pulse the Servo allows
        min_angle: The minimum degree the Servo can rotate to
        max_angle: The maximum degree the Servo can rotate to
        default_angle: The rest(initial)position of the servo.
        name: The name of the servo
    """

    def __init__(self,
                 servo_id,
                 min_pulse,
                 max_pulse,
                 min_angle=None,
                 max_angle=None,
                 default_angle=None,
                 name="servo",
                 disabled=False
                 ):
        
        self.disabled = disabled
        self.servo_id = servo_id
        # servo IDs start from 0, and because there are only 16 channels on a hat, the actual channel is ID mod 16.
        # also, the hat number is ID // 16 (round-down division).
        # Example: ID=20, 20 % 16 = 4, 20 // 16 = 1. Channel 4, hat 1.
        self.channel_id = int(servo_id % 16)
        self.shield_id = int(servo_id // 16)
        self.min_pulse = min_pulse
        self.max_pulse = max_pulse
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.default_angle = default_angle
        self.name = name
        # sanity checking, in case of typos in the config json
        if self.disabled:
            print("Warning: servo '%s' is disabled" % name)
        else:
            # currently we do not prevent either degrees or pwm from being upside-down, this might change in future
            # assert min_pulse <= max_pulse
            # assert min_angle <= max_angle
            assert 1 <= max_pulse <= 4096
            assert 1 <= min_pulse <= 4096
            assert 0 <= servo_id <= 47  # maximum is 3 hats, only 48 channels possible
            assert -360 <= min_angle <= 360
            assert -360 <= max_angle <= 360
            assert min(min_angle, max_angle) <= default_angle <= max(min_angle, max_angle)
        
        # running/idle flags: normal Events can only wait for a rising edge, if I want to wait for a falling edge, i need to
        # set up a complementary system like this. also they're mostly being used as flags, not as "events", but whatever.
        self.state_flag_running = threading.Event()
        self.state_flag_idle = threading.Event()
        self.state_flag_idle.set()
        # i want setting one/clearing other to be an indivisible atomic operation so it should have a lock object just in case
        self.state_flag_lock = threading.Lock()
        # the list of frames that the leg thread is consuming as the leg object is adding onto
        self.frame_queue = []
        # locking object to ensure no collisions happen around the frame queue
        self.frame_queue_lock = threading.Lock()
        # these variables track the state of the servo at any given instant
        self.curr_angle = 0.0
        self.curr_pwm = 0
        self.curr_on = False
        # locking object to ensure no collisions happen around self.curr_angle/self.curr_pwm/self.curr_on
        # might not be necessary but couldn't hurt, technically both the servo thread and servo object can write into them
        self.curr_state_lock = threading.Lock()

        # create and launch the thread for this servo
        # note: this MUST be daemon type because the thread is designed to run forever...
        # the only way to stop it is by stopping its parent, which means it must be a daemon!
        # it will be able to access all of this servos's other member variables and functions
        self.framethread_name = "framethread_" + self.name
        self.framethread = threading.Thread(name=self.framethread_name,
                                            target=ft.frame_thread_func, args=(self, DEBUG_SERVO_ID))
        self.framethread.daemon = True

        # start the thread, this should be the 2nd last operation of __init__
        self.framethread.start()
        # the servo begins as "off" until explicitly told to initialize it
        self.off()
        
    def __str__(self):
        # self.name, self.id, self.channel_id, self.shield_id, self.min_pulse, self.max_pulse, self.min_angle, self.max_angle, self.default_angle, self.disabled
        # curr_angle, curr_pwm, curr_on
        s = "name={}, servo_id={}, channel_id={}, shield_id={}, min_pulse={}, max_pulse={}, min_angle={}, max_angle={}, default_angle={}, disabled={}\ncurr_angle={}, curr_pwm={}, curr_on={}"
        return s.format(self.name, self.servo_id, self.channel_id, self.shield_id, self.min_pulse, self.max_pulse,
                        self.min_angle, self.max_angle, self.default_angle, self.disabled,
                        self.curr_angle, self.curr_pwm, self.curr_on)



    def rotate(self, degree):
        """
        Rotate to the specified degrees
        non-threading method of controlling the servo
        perform safety clamp before passing to do_set_angle
        """
        # todo: rename
        degree_safe = self.degrees_clamp(degree)
        # calling the non-threading control function should cancel any pending threading events
        self.abort()
        try:
            self.do_set_angle(degree_safe)
        except ValueError as exception:
            print(exception)
            print("Could not rotate {} to {} degree").format(self.name, degree)


    def rotate_thread(self, degree, durr):
        """
        Rotate to the specified degrees gradually over the specified duration
        threading method of controlling the servo
        perform safety clamp, interpolate, append frames to frame queue, and sets running flag
        the thread will jump in with "do_set_servo_angle" when it is the correct time
        """
        # safety clamp
        dest = self.degrees_clamp(degree)
        
        # if there is a queued interpolation frame, interpolate from the final frame in the queue to the desired pose.
        # otherwise, interpolate from current position.
        curr = None
        with self.frame_queue_lock:
            if len(self.frame_queue) > 0:
                curr = self.frame_queue[-1][0]
        if curr is None:  # "else" but outside of the lock block
            # floats are always copied, not referenced
            curr = self.curr_angle

        if self.servo_id == DEBUG_SERVO_ID:
            print("servo_%s: interp from deg %d to %d over %f" % (self.name, curr, dest, durr))
        
        # run interpolation
        interp_list = ft.interpolate(dest, curr, durr)
    
        # add new frames onto the END of the frame queue (with lock)
        with self.frame_queue_lock:
            # concatenate two lists with +
            self.frame_queue = self.frame_queue + interp_list
            if self.servo_id == DEBUG_SERVO_ID:
                print("servo_%s: add %d frames, new length %d" % (self.name, len(interp_list), len(self.frame_queue)))
    
        with self.state_flag_lock:
            # clear "sleeping" event, does not trigger anything (note: clear before set)
            self.state_flag_idle.clear()
            # set the "running" event, this will trigger the thread to begin consuming frames
            # note: do this unconditionally! no harm in setting an already set flag
            self.state_flag_running.set()


    def do_set_angle(self, degree):
        """
        take angle after clamp, already known to be safe value
        convert to pwm, set actual pwm, also set "self.curr_x" values
        called by both threading and non-threading approaches
        """
        
        if self.disabled:
            return
        
        pulse = self.degrees_to_pulse(degree)

        with self.curr_state_lock:
            self.curr_on = True
            self.curr_angle = degree
            self.curr_pwm = pulse
            set_pwm(self.shield_id, self.channel_id, pulse)

    def initialize(self):
        """ Move servo to defult position """
        print("init name = {}, default_angle = {}".format(self.name, self.default_angle))
        self.rotate(self.default_angle)

    def off(self):
        """ setting PWM to 0 cuts power to the servo and makes it malleable """
        if self.disabled:
            return

        # abort so it doesn't wake up after turning off until i explicitly tell it to wake up
        self.abort()
        try:
            with self.curr_state_lock:
                self.curr_on = False
                #self.curr_angle = None
                self.curr_pwm = 0
                set_pwm(self.shield_id, self.channel_id, 0)
        except ValueError as exception:
            print(exception)
            print("Could not turn off", self.name)


    # clear the frame queue to stop any currently-pending movements.
    def abort(self):
        with self.frame_queue_lock:
            self.frame_queue = []





    def degrees_clamp(self, degree):
        # clamp for safety
        degree_safe = float(bidirectional_clamp(degree, self.min_angle, self.max_angle))
        # warn if clamping actually occurred
        if degree != degree_safe:
            print("Degree {} is out of range, clamping to safe value {}".format(degree, degree_safe))
        return degree_safe
    
    def degrees_to_pulse(self, degree):
        """ Map degree input value to a pulse length output value """
        # perform actual mapping
        pulse = int(linear_map(degree, self.min_angle, self.max_angle, self.min_pulse, self.max_pulse))
        return pulse

# removed setters & getters cuz parameters are not redefined after initialization, therefore they are useless
