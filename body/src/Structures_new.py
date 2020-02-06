#!/bin/python
import time

# members of each class:
# finger: bend, bend_max, straighten_max
# hand: straighten_all_fingers, make_fist, move_all_fingers
# wrist: rotate
# forearm:
# shoulder: flex/extend, abduction_up/abduction_down, rotation_internal/rotation_external, rotation_up/rotation_down,
# arm:

# torso: lean
# head: move_x, move_y



# TODO: probably too much hierarchy? each level adds little/nothing but naming confusion
# right_wrist = right_wrist
# right_hand = right_pinky + right_ring + right_mid + right_index + right_thumb
# right_forearm = right_hand + right_wrist
# right_shoulder = right_shoulder_flexion + right_shoulder_abduction + right_shoulder_rotation_x + right_shoulder_rotation_y
# right_arm = right_forearm + right_shoulder


class Finger(object):
    """
    This class represents an Inmoov Finger.
    """

    def __init__(self, servo):
        if servo is None:
            raise Exception("Could not initialize Finger")

        self.servo = servo

    def bend_max(self):
        """
        Bend the Finger the max amount
        """
        self.servo.rotate(self.servo.min_degree)

    def bend(self, degree):
        """
        Bend the Finger
        """
        self.servo.rotate(degree)

    def straighten_max(self):
        """
        This is to make the finger straight.
        """
        self.servo.rotate(self.servo.max_degree)
        
    def initialize(self):
        self.servo.initialize()

    def off(self):
        self.servo.off()


class Hand(object):
    """ This class represents an Inmoov Hand. """

    def __init__(self, pinky_finger, ring_finger, mid_finger, index_finger, thumb_finger):
        """ Build an Inmoov Hand """
        if pinky_finger is None or ring_finger is None or mid_finger is None or index_finger is None or thumb_finger is None:
            raise Exception('Could not initialize Hand')
        self.pinky_finger = pinky_finger
        self.ring_finger = ring_finger
        self.mid_finger = mid_finger
        self.index_finger = index_finger
        self.thumb = thumb_finger

    def straighten_all_fingers(self):
        """ Straighten all fingers for waving/high-fiving/etc. """
        self.pinky_finger.straighten_max()
        self.ring_finger.straighten_max()
        self.mid_finger.straighten_max()
        self.index_finger.straighten_max()
        self.thumb.straighten_max()

    def make_fist(self):
        """ Bend all fingers in to make a fist """
        self.pinky_finger.bend_max()
        self.ring_finger.bend_max()
        self.mid_finger.bend_max()
        self.index_finger.bend_max()
        self.thumb.bend_max()

    def move_fingers(self, pinky_deg=None, ring_deg=None, mid_deg=None,
                     index_deg=None, thumb_deg=None):
        """ Bend the Fingers that have values sent in."""

        if pinky_deg is not None:
            self.pinky_finger.bend(pinky_deg)
        if ring_deg is not None:
            self.ring_finger.bend(ring_deg)
        if mid_deg is not None:
            self.mid_finger.bend(mid_deg)
        if index_deg is not None:
            self.index_finger.bend(index_deg)
        if thumb_deg is not None:
            self.thumb.bend(thumb_deg)

    def initialize(self):
        self.pinky_finger.initialize()
        self.ring_finger.initialize()
        self.mid_finger.initialize()
        self.index_finger.initialize()
        self.thumb.initialize()

    def off(self):
        """ Turn off all fingers off"""
        self.pinky_finger.off()
        self.ring_finger.off()
        self.mid_finger.off()
        self.index_finger.off()
        self.thumb.off()


class Wrist(object):
    """ This class represents an Inmoov Wrist """

    # TODO: Pull apart Inmoov's forearm to find out servo models for a Wrist. These values are just copied from the HS-805BB Servo.

    def __init__(self, servo):
        """ Set the Servo for this Wrist """

        if servo is None:
            raise Exception("Could not initiate wrist")
        self.servo = servo

    def rotate(self, degree):
        """ Rotate this Wrist the desired degree """
        self.servo.rotate(degree)

    def initialize(self):
        self.servo.initialize()

    def off(self):
        """ Turn off all fingers off"""
        self.servo.off()


class Forearm(object):
    """
    This class represents an Inmoov Forearm
    """
    
    def __init__(self, hand, wrist):
        """
        Build an Inmoov Forearm
        """
        if hand is None or wrist is None:
            raise Exception("Could not build a forearm")
        self.wrist = wrist
        self.hand = hand
    
    def initialize(self):
        self.wrist.initialize()
        self.hand.initialize()
    
    def off(self):
        self.wrist.off()
        self.hand.off()


class Shoulder(object):
    """ This class represents an Inmoov Shoulder """
    
    def __init__(self, flexion_servo, abduction_servo, rotation_servo_x, rotation_servo_y):
        """ Build an Inmoov Shoulder """
        if flexion_servo is None or abduction_servo is None or rotation_servo_x is None or rotation_servo_y is None:
            raise Exception("Could not initiate shoulder")
        self.flexion_servo = flexion_servo
        self.abduction_servo = abduction_servo
        self.rotation_servo_x = rotation_servo_x
        self.rotation_servo_y = rotation_servo_y
        
    # TODO: DEFINITELY want to reduce/rename these function
    
    def flex(self, degree):
        self.flexion_servo.rotate(degree)
    
    def extend(self, degree):
        self.flexion_servo.rotate(degree)
    
    def abduction_up(self, degree):
        self.abduction_servo.rotate(degree)
    
    def abduction_down(self, degree):
        self.abduction_servo.rotate(degree)
    
    def rotation_internal(self, degree):
        self.rotation_servo_y.rotate(degree)
    
    def rotation_external(self, degree):
        self.rotation_servo_y.rotate(degree)
    
    def rotation_up(self, degree):
        self.rotation_servo_x.rotate(degree)
    
    def rotation_down(self, degree):
        self.rotation_servo_x.rotate(degree)

    def initialize(self):
        # Make the arm in down postion
        self.flexion_servo.initialize()
        self.abduction_servo.initialize()
        self.rotation_servo_x.initialize()
        self.rotation_servo_y.initialize()

    def off(self):
        self.rotation_servo_x.off()
        self.rotation_servo_y.off()
        self.abduction_servo.off()
        self.flexion_servo.off()


class Arm(object):
    """
    This class represents an Inmoov Arm
    """

    def __init__(self, forearm, shoulder):
        """
        Build an Inmoov Arm
        """
        if forearm is None or shoulder is None:
            raise Exception("Could not build a arm")
        self.forearm = forearm
        self.shoulder = shoulder

    def initialize(self):
        self.forearm.initialize()
        time.sleep(1)
        self.shoulder.initialize()

    def off(self):
        self.forearm.off()
        self.shoulder.off()


class Torso(object):
    """ This class is used to control Inmoov's Torso """
    # TODO these need to be very precisely calibrated since they are fighting eachother

    def __init__(self, left_servo, right_servo):
        """
        Initialize all of Inmoov's Torso variables.
        """
        if left_servo is None or right_servo is None:
            raise Exception("Could not initialize Torso")
        self.l_servo = left_servo
        self.r_servo = right_servo

    def lean(self, degrees):
        """
        Make Inmoov lean based on the specified degree.
        - -90 degrees leans Inmoov all the way right.
        -   0 degrees centers Inmoov's Torso.
        -  90 degrees leans Inmoov all the way left.
        """
        self.l_servo.rotate(degrees)
        self.r_servo.rotate(degrees)

    def initialize(self):
        self.l_servo.initialize()
        self.r_servo.initialize()

    def off(self):
        self.l_servo.off()
        self.r_servo.off()


class Head(object):
    """
    This class is used to control the motion of Inmoov's head.
    - TODO: Add mouth and eye control.
    """
    
    def __init__(self, x_servo, y_servo, jaw):
        """ Initialize all Head Servos and look forward """
        if x_servo is None or y_servo is None or jaw is None:
            raise Exception("Could not initialize Head")
        self.x_servo = x_servo
        self.y_servo = y_servo
        self.jaw = jaw
    
    def move_y(self, degrees):
        """
        Move head to y-axis to the degree postion.
        - -90 degrees places Inmoov's chin to his chest.
        -   0 degrees makes Inmoov look forward.
        -  90 degrees makes Inmoov look up.
        """
        self.y_servo.rotate(degrees)
    
    def move_x(self, degrees):
        """
        Move head to x-axis to the degree postion.
        - -90 degrees moves Inmoov's head all the way right.
        -   0 degrees makes Inmoov look forward.
        -  90 degrees moves Inmoov's head all the way left.
        """
        self.x_servo.rotate(degrees)
        
    def move_jaw(self, degrees):
        self.jaw.rotate(degrees)

    def initialize(self):
        # Make head look straight forward x and y-axis
        self.x_servo.initialize()
        self.y_servo.initialize()

    def off(self):
        self.x_servo.off()
        self.y_servo.off()

