#!/bin/python
"""
This module are Inmoov!

Authors:
    Brett Creeley
    Matty Baba Allos
"""
import json
import time
from Servo import Servo
from Structures_new import *

# all of the config data (min angle, max angle, channel, etc) is stored in this JSON file
INMOOV_FILE = "inmoov_servo.json"


'''
notes
each servo has .off() member and .initialize() member
each structure has .off() and .initialize() members that pass the call to their servo(s)
does .off() actually cut power, or does it drive the servo to position 0? must test!

Servo.py defines a servo object, needs work but its' just fine
    todo: add "current position" member and threading system
all calibration & channel_id data is in inmoov_servo.json, odd storage method but no reason to change it yet
Pwm_Interface.py inits the PWM hats, handles the locking, & defines "set_pwm()"
    todo: rename, change to use "default/builtin" adafruit libs instead of these local things
body.py is a top-level thing that sets up some ROS stuff
Structures_new.py just puts all the ways of grouping servos into one file, they dont add anything except hierarchy

'''
# TODO: make better hierarchy
# TODO: verify right wrist servo works


# right fingers: pinky=0 thru thumb=4
# head: y=13, x=14, jaw=15

# note: shoulder_lift_out max current draw is nearly 1.2A, thats almost our entire budget for a board...

class Inmoov(object):
    """
    This class are Inmoov!
    This class contains all the torso servos accessed as members to make it move certain ways
    """
    def __init__(self):
        """
        Build most of Inmoov's parts.
        """
        print("begin InMoov init")
        
        self.all_servos = []
        
        # open the json config file
        # run the "parse" function on each object it reads from file, therefore filling the "servos" list
        with open(INMOOV_FILE) as json_file:
            json.load(json_file,object_hook=self.parse)
            
        self.all_servos.sort(key=lambda x: x.servo_id)
        
        ####################################
        # store the actual servo objects as Inmoov member in addition to the bodypart structure hierarchy members
        # 25 total servos
        self.head_x = self.find_servo_by_name("head_x")
        self.head_y = self.find_servo_by_name("head_y")
        self.jaw = self.find_servo_by_name("jaw")
        self.left_torso = self.find_servo_by_name("left_torso")
        self.right_torso = self.find_servo_by_name("right_torso")
        self.left_wrist = self.find_servo_by_name("left_wrist")
        self.left_pinky = self.find_servo_by_name("left_pinky")
        self.left_ring = self.find_servo_by_name("left_ring")
        self.left_mid = self.find_servo_by_name("left_mid")
        self.left_index = self.find_servo_by_name("left_index")
        self.left_thumb = self.find_servo_by_name("left_thumb")
        self.left_elbow = self.find_servo_by_name("left_elbow")
        self.left_shoulder_lift_out = self.find_servo_by_name("left_shoulder_lift_out")
        self.left_shoulder_lift_front = self.find_servo_by_name("left_shoulder_lift_front")
        self.left_arm_rotate = self.find_servo_by_name("left_arm_rotate")
        self.right_wrist = self.find_servo_by_name("right_wrist")
        self.right_pinky = self.find_servo_by_name("right_pinky")
        self.right_ring = self.find_servo_by_name("right_ring")
        self.right_mid = self.find_servo_by_name("right_mid")
        self.right_index = self.find_servo_by_name("right_index")
        self.right_thumb = self.find_servo_by_name("right_thumb")
        self.right_elbow = self.find_servo_by_name("right_elbow")
        self.right_shoulder_lift_out = self.find_servo_by_name("right_shoulder_lift_out")
        self.right_shoulder_lift_front = self.find_servo_by_name("right_shoulder_lift_front")
        self.right_arm_rotate = self.find_servo_by_name("right_arm_rotate")

        ####################################
        # hierarchical structures arranging the same servos listed above
        
        # build head, this structure makes sense
        # head = head_x + head_y + jaw
        self.head = Head(self.head_x, self.head_y, self.jaw)

        # "Torso" object, this structure also makes sense
        # torso = left_torso + right_torso
        self.torso = Torso(self.left_torso, self.right_torso)

        self.left_hand = Hand(
            Finger(self.left_pinky),
            Finger(self.left_ring),
            Finger(self.left_mid),
            Finger(self.left_index),
            Finger(self.left_thumb)
        )
        
        # left arm
        # better hierarchy: arm = hand(5) + wrist(1) + elbow(1) + twist(1) + shoulder(2)
        self.left_arm = Arm(self.left_hand, self.left_wrist, self.left_elbow, self.left_arm_rotate,
                            self.left_shoulder_lift_front, self.left_shoulder_lift_out)

        self.right_hand = Hand(
            Finger(self.right_pinky),
            Finger(self.right_ring),
            Finger(self.right_mid),
            Finger(self.right_index),
            Finger(self.right_thumb)
        )

        # right arm
        # better hierarchy: arm = hand(5) + wrist(1) + elbow(1) + twist(1) + shoulder(2)
        self.right_arm = Arm(self.right_hand, self.right_wrist, self.right_elbow, self.right_arm_rotate,
                             self.right_shoulder_lift_front, self.right_shoulder_lift_out)

        # should probably just init to "off" then explicitly call "initialize" outside this
        self.off()
        print("done with InMoov init")

    def parse(self, obj):
        # build a Servo object out of the data read from the JSON
        # then append it into the global "servos" list
        if "body_part" not in obj:
            raise Exception("Could not parse JSON object")
    
        if "disabled" in obj and obj["disabled"] == True:
            d = True
        else:
            d = False
    
        self.all_servos.append(Servo(
            obj["id"],
            obj["min_pulse"],
            obj["max_pulse"],
            obj["min_angle"],
            obj["max_angle"],
            obj["default_angle"],
            obj["body_part"],
            disabled=d
        ))

    def find_servo_by_name(self, name: str) -> Servo:
        # in the global list "servos" find the Servo obj with x.name matching given name
        for i in self.all_servos:
            if name == i.name:
                return i
        print("ERR: FAIL TO FIND")
        return None

    def set_servo_ros(self, cmd_string: str):
        # designed to interface with ROS, receive a string encoding the servo + position, set the relevant servo
        # this way we can run the interactive poser on a laptop or whatever
        # input string format: "{name}!{degrees}"
        name, deg = cmd_string.split("!")
        s = self.find_servo_by_name(name)
        if s is None:
            return
        try:
            d = float(deg)
        except ValueError as ve:
            print(ve)
            print(cmd_string)
            print("fail to parse given degree")
            return
        s.rotate(d)

    def do_motion(self, motion_id):
        """
        Make InMoov do one of these motions
        """
        if motion_id == 0:
            self.wave()
        elif motion_id == 1:
            self.point()
            time.sleep(5)
            self.wave()
        elif motion_id == 2:
            self.initialize()


    def wave(self):
        self.left_arm.hand.open_all()
        self.left_arm.lift_front.rotate(-20) # lift_front
        self.left_arm.twist.rotate(60) # arm_twist
        self.left_arm.lift_out.rotate(-90) # lift_out
        time.sleep(2)
        self.left_arm.lift_out.rotate(60)
        time.sleep(0.5)
        time.sleep(2)
        self.left_arm.lift_out.rotate(90)
        time.sleep(1.5)
        self.left_arm.lift_out.rotate(0)
        time.sleep(2)
        self.left_arm.lift_out.rotate(90)
        time.sleep(1.5)
        self.left_arm.lift_out.rotate(0)
    
        self.left_arm.lift_front.rotate(-20)
        self.left_arm.elbow.rotate(90) # elbow
        self.left_arm.twist.rotate(60)

    def point(self):
        self.left_arm.lift_front.rotate(-20)
        self.left_arm.twist.rotate(60)
        time.sleep(3)

        self.left_arm.twist.rotate(90)
        self.left_arm.hand.close_all()
        self.left_arm.lift_front.rotate(60)
        self.left_arm.hand.open_all()
    def thumbs_down(self):
        pass
    def goodbye(self):
        pass

    def initialize(self):
        """ initializes all servos in InMoov, order/delays might be important """
        self.head.initialize()
        self.left_arm.initialize()
        self.right_arm.initialize()
        self.torso.initialize()

    def off(self):
        """ Turns off all servos in InMoov """
        for s in self.all_servos:
            s.off()
