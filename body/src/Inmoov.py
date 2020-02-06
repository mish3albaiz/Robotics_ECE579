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
# from _Arm import Arm
# from _Head import Head
# from _Hand import Hand
# from _Forearm import Forearm
# from _Wrist import Wrist
# from _Finger import Finger
# from _Shoulder import Shoulder
# from _Torso import Torso
from Structures_new import *

# all of the config data (min angle, max angle, channel, etc) is stored in this JSON file
INMOOV_FILE = "inmoov_servo.json"
servos = []


'''
notes
each servo has .off() member and .initialize() member
each structure has .off() and .initialize() members that pass the call to their servo(s)
does .off() actually cut power, or does it drive the servo to position 0? must test!

Servo.py defines a servo object, needs work but its' just fine
    todo: add "current position" member and threading system
all calibration & channel data is in inmoov_servo.json, odd storage method but no reason to change it yet
Config.py inits the PWM hats, handles the locking, & defines "set_pwm()"
    todo: rename, change to use "default/builtin" adafruit libs instead of these local things
body.py is a top-level thing that sets up some ROS stuff
Structures_new.py just puts all the ways of grouping servos into one file, they dont add anything except hierarchy

'''
# TODO: make better hierarchy
# TODO: verify right wrist servo works


def parse(obj):
    # build a Servo object out of the data read from the JSON
    # then append it into the global "servos" list
    if "body_part" not in obj:
        raise Exception("Could not parse JSON object")

    if "disabled" in obj and obj["disabled"] == True:
        d = True
    else:
        d = False
        
    servos.append(Servo(
        obj["id"],
        obj["min_pulse"],
        obj["max_pulse"],
        obj["min_degree"],
        obj["max_degree"],
        obj["default_angle"],
        obj["body_part"],
        disabled=d
        ))


def find_servo_in_list(name):
    # in the global list "servos" find the Servo obj with x.name matching given name
    for i in servos:
        if name == i.name:
            return i
    print("ERR: FAIL TO FIND")
    return None

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
        
        # open the json config file
        # run the "parse" function on each object it reads from file, therefore filling the "servos" list
        with open(INMOOV_FILE) as json_file:
            json.load(json_file,object_hook=parse)
            
        self.all_servos = servos
        self.all_servos.sort(key=lambda x: x.servo_id)
        
        ####################################
        # store the actual servo objects as Inmoov member in addition to the bodypart structure hierarchy members
        # 25 total servos
        self.head_x = find_servo_in_list("head_x")
        self.head_y = find_servo_in_list("head_y")
        self.jaw = find_servo_in_list("jaw")
        self.left_torso = find_servo_in_list("left_torso")
        self.right_torso = find_servo_in_list("right_torso")
        self.left_wrist_serv = find_servo_in_list("left_wrist")
        self.left_pinky = find_servo_in_list("left_pinky")
        self.left_ring = find_servo_in_list("left_ring")
        self.left_mid = find_servo_in_list("left_mid")
        self.left_index = find_servo_in_list("left_index")
        self.left_thumb = find_servo_in_list("left_thumb")
        self.left_shoulder_flexion = find_servo_in_list("left_shoulder_flexion")
        self.left_shoulder_abduction = find_servo_in_list("left_shoulder_abduction")
        self.left_shoulder_rotation_x = find_servo_in_list("left_shoulder_rotation_x")
        self.left_shoulder_rotation_y = find_servo_in_list("left_shoulder_rotation_y")
        self.right_wrist_serv = find_servo_in_list("right_wrist")
        self.right_pinky = find_servo_in_list("right_pinky")
        self.right_ring = find_servo_in_list("right_ring")
        self.right_mid = find_servo_in_list("right_mid")
        self.right_index = find_servo_in_list("right_index")
        self.right_thumb = find_servo_in_list("right_thumb")
        self.right_shoulder_flexion = find_servo_in_list("right_shoulder_flexion")
        self.right_shoulder_abduction = find_servo_in_list("right_shoulder_abduction")
        self.right_shoulder_rotation_x = find_servo_in_list("right_shoulder_rotation_x")
        self.right_shoulder_rotation_y = find_servo_in_list("right_shoulder_rotation_y")

        ####################################
        # hierarchical structures arranging the same servos listed above
        
        # build head, this structure makes sense
        # head = head_x + head_y + jaw
        self.head = Head(self.head_x, self.head_y, self.jaw)

        # "Torso" object, this structure also makes sense
        # torso = left_torso + right_torso
        self.torso = Torso(self.left_torso, self.right_torso)

        # left arm
        # better hierarchy: arm = hand(5) + wrist(1) + elbow(1) + shoulder(4?)
        # flexion = elbow
        # abduction = ?forward?
        # rotation_x = "lifts arm up"
        # rotation_y = "rotate arm around itself"
        # TODO: possibly too much hierarchy?
        # left_wrist = left_wrist
        # left_hand = left_pinky + left_ring + left_mid + left_index + left_thumb
        # left_forearm = left_hand + left_wrist
        # left_shoulder = left_shoulder_flexion + left_shoulder_abduction + left_shoulder_rotation_x + left_shoulder_rotation_y
        # left_arm = left_forearm + left_shoulder
        self.left_wrist = Wrist(self.left_wrist_serv)
        self.left_hand = Hand(
            Finger(self.left_pinky),
            Finger(self.left_ring),
            Finger(self.left_mid),
            Finger(self.left_index),
            Finger(self.left_thumb)
        )
        self.left_forearm = Forearm(self.left_hand, self.left_wrist)
        self.left_shoulder = Shoulder(
            self.left_shoulder_flexion,
            self.left_shoulder_abduction,
            self.left_shoulder_rotation_x,
            self.left_shoulder_rotation_y)

        self.left_arm = Arm(self.left_forearm, self.left_shoulder)

        # Right side
        # TODO: possibly too much hierarchy?
        # right_wrist = right_wrist
        # right_hand = right_pinky + right_ring + right_mid + right_index + right_thumb
        # right_forearm = right_hand + right_wrist
        # right_shoulder = right_shoulder_flexion + right_shoulder_abduction + right_shoulder_rotation_x + right_shoulder_rotation_y
        # right_arm = right_forearm + right_shoulder
        self.right_wrist = Wrist(self.right_wrist_serv)
        self.right_hand = Hand(
            Finger(self.right_pinky),
            Finger(self.right_ring),
            Finger(self.right_mid),
            Finger(self.right_index),
            Finger(self.right_thumb)
        )
        self.right_forearm = Forearm(self.right_hand, self.right_wrist)
        self.right_shoulder = Shoulder(
            self.right_shoulder_flexion,
            self.right_shoulder_abduction,
            self.right_shoulder_rotation_x,
            self.right_shoulder_rotation_y)

        self.right_arm = Arm(self.right_forearm, self.right_shoulder)

        # should probably just init to "off" then explicitly call "initialize" outside this
        self.initialize()
        print("done with InMoov init")


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
        self.left_arm.forearm.hand.off()
        self.left_arm.shoulder.rotation_up(-20)
        self.left_arm.shoulder.rotation_internal(60)
        self.left_arm.shoulder.abduction_up(-90)
        time.sleep(2)
        self.left_arm.shoulder.abduction_up(60)
        time.sleep(0.5)
        time.sleep(2)
        self.left_arm.shoulder.abduction_up(90)
        time.sleep(1.5)
        self.left_arm.shoulder.abduction_up(0)
        time.sleep(2)
        self.left_arm.shoulder.abduction_up(90)
        time.sleep(1.5)
        self.left_arm.shoulder.abduction_up(0)
    
        self.left_arm.shoulder.rotation_up(-20)
        self.left_arm.shoulder.flex(90)
        self.left_arm.shoulder.rotation_internal(60)

    def point(self):
        self.left_arm.shoulder.rotation_up(-20)
        self.left_arm.shoulder.rotation_internal(60)
        time.sleep(3)

        self.left_arm.shoulder.rotation_internal(90)
        self.left_arm.forearm.hand.make_fist()
        self.left_arm.shoulder.rotation_up(60)
        self.left_arm.forearm.hand.straighten_all_fingers()
    def thumbs_down(self):
        pass
    def goodbye(self):
        pass

    def initialize(self):
        """initializes InMoov"""
        self.head.initialize()
        self.left_arm.initialize()
        self.right_arm.initialize()
        self.torso.initialize()

    def off(self):
        """Turns InMoov off"""
        self.head.off()
        self.left_arm.off()
        self.right_arm.off()
        self.torso.off()
