"""
This module are Inmoov!

Authors:
    Brian Henson
"""
import json
import time
from Servo import Servo
from Structures_new import *
import sys
from os.path import join, dirname
whereami = dirname(__file__)
# guidir = join(whereami, "../gui/")
# sys.path.append(guidir)
import animation_executor

# all of the config data (min angle, max angle, channel, etc) is stored in this JSON file
# turns that filename into the absolute location of the json file
inmoov_json_absolute = join(whereami, "../json/inmoov_servo.json")

# TODO: better comments describing the whole file and the whole package structure

class Inmoov(object):
    """
    This class are Inmoov!
    This class contains all the torso servos accessed as members to make it move certain ways
    """
    def __init__(self):
        """
        Build all of Inmoov's parts.
        """
        print("begin InMoov bootup")
        
        self.all_servos = []

        
        # open the json config file
        # run the "parse" function on each object it reads from file, therefore filling the "servos" list
        with open(inmoov_json_absolute) as json_file:
            json.load(json_file,object_hook=self.parse)
            
        # assert that all servos have unique ID, to prevent channel collisions
        assert len(set([x.servo_id for x in self.all_servos])) == len(self.all_servos)
        # assert that all servos have unique names, to prevent find_servo_by_name collisions
        assert len(set([x.name for x in self.all_servos])) == len(self.all_servos)

        ####################################
        # read the animations and poses json files, only needs read once at bootup
        # when this is running on the bot being controlled over ROS, the files will not change
        # therefore they do not need to be read every time they are called
        animation_executor.update_animations()
        animation_executor.update_poses()
        
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
        print("done with InMoov bootup, all servos currently off")

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

    def find_servo_by_name(self, name):
        # in the global list "servos" find the Servo obj with x.name matching given name
        for i in self.all_servos:
            if name == i.name:
                return i
        print("FAIL TO FIND: SERVO '%s'" % name)
        return None

    def inmoov_ros_command(self, cmd_string):
        # designed to interface with ROS, receive a string encoding the servo + position, set the relevant servo
        # this way we can run the interactive poser on a laptop or whatever
        # input string format:
        # "servo!{name}!{degrees}" or
        # "servo_thread!{name}!{degrees}!{time}" or
        # "init!" or
        # "off!" or
        # "pose!{name}" or
        # "anim!{name}"
        splitted = cmd_string.split("!")
        if splitted[0] == "servo":
            dummy, name, deg = splitted
            s = self.find_servo_by_name(name)
            if s is None:
                # already prints message for "fail to find servo"
                return
            try:
                d = float(deg)
            except ValueError:
                print("fail to parse given degree", cmd_string)
                return
            s.rotate(d)
        elif splitted[0] == "servo_thread":
            dummy, name, deg, mytime = splitted
            s = self.find_servo_by_name(name)
            if s is None:
                # already prints message for "fail to find servo"
                return
            try:
                d = float(deg)
            except ValueError:
                print("fail to parse given degree", cmd_string)
                return
            try:
                t = float(mytime)
            except ValueError:
                print("fail to parse given time", cmd_string)
                return
            s.rotate_thread(d, t)
        elif splitted[0] == "init":
            self.initialize()
            return
        elif splitted[0] == "off":
            self.off()
            return
        elif splitted[0] == "pose":
            dummy, name = splitted
            # execute the specified pose
            animation_executor.do_pose(self, name)
            return
        elif splitted[0] == "anim":
            dummy, name = splitted
            # execute the specified animation
            animation_executor.do_animation(self, name)
            return
        else:
            print("received unsupported command type '%s'" % splitted[0])
        return
    
    '''
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
    '''

    def initialize(self):
        """ initializes all servos in InMoov, order/delays might be important """
        print("inmoov all init")
        self.head.initialize()
        self.left_arm.initialize()
        self.right_arm.initialize()
        self.torso.initialize()

    def off(self):
        """ Turns off all servos in InMoov """
        print("inmoov all off")
        for s in self.all_servos:
            s.off()
