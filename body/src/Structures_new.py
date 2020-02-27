#!/bin/python
import time
from Servo import Servo

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
# right_shoulder = right_elbow + right_shoulder_lift_out + right_shoulder_lift_front + right_arm_rotate
# right_arm = right_forearm + right_shoulder


class Finger(object):
    """ This class represents an Inmoov Finger. """

    def __init__(self, servo):
        if servo is None:
            raise Exception("Could not initialize Finger")
        self.servo = servo

    def bend(self, degree):
        """ Alias for .rotate() on this servo, 0=close, 100=open """
        self.servo.rotate(degree)

    def open(self):
        """ Open/straighten/point """
        self.servo.rotate(self.servo.max_angle)

    def close(self):
        """ Close/curl/grip """
        self.servo.rotate(self.servo.min_angle)

    def initialize(self):
        self.servo.initialize()

    def off(self):
        self.servo.off()


class Hand(object):
    """ This class represents an Inmoov Hand. """

    def __init__(self, pinky, ring, mid, index, thumb):
        """ Build an Inmoov Hand """
        if pinky is None or ring is None or mid is None or index is None or thumb is None:
            raise Exception('Could not initialize Hand')
        self.pinky = pinky
        self.ring = ring
        self.mid = mid
        self.index = index
        self.thumb = thumb

    def bend(self, pinky=None, ring=None, mid=None, index=None, thumb=None):
        """ Bend the Fingers that have values sent in. Pinky thru Thumb. 0=close, 100=open """
        if pinky is not None:
            self.pinky.bend(pinky)
        if ring is not None:
            self.ring.bend(ring)
        if mid is not None:
            self.mid.bend(mid)
        if index is not None:
            self.index.bend(index)
        if thumb is not None:
            self.thumb.bend(thumb)
            
    def bend_all(self, deg):
        """ alias for calling .bend() will all finger values the same """
        v = [deg] * 5
        self.bend(*v)

    def open_all(self):
        """ Open/straighten/point all fingers for waving/high-fiving/etc. """
        self.pinky.open()
        self.ring.open()
        self.mid.open()
        self.index.open()
        self.thumb.open()

    def close_all(self):
        """ Close/curl/grip """
        self.pinky.close()
        self.ring.close()
        self.mid.close()
        self.index.close()
        self.thumb.close()

    def initialize(self):
        self.pinky.initialize()
        self.ring.initialize()
        self.mid.initialize()
        self.index.initialize()
        self.thumb.initialize()

    def off(self):
        """ De-power all fingers """
        self.pinky.off()
        self.ring.off()
        self.mid.off()
        self.index.off()
        self.thumb.off()


'''
class Wrist(object):
    """ This class represents an Inmoov Wrist """

    # TODO: Pull apart Inmoov's forearm to find out servo models for a Wrist. These values are just copied from the HS-805BB Servo.

    def __init__(self, servo):
        """ Set the Servo for this Wrist """

        if servo is None:
            raise Exception("Could not initiate wrist")
        self.servo = servo

    def rotate(self, degree):
        """ Alias for .rotate() on this servo """
        self.servo.rotate(degree)

    def initialize(self):
        self.servo.initialize()

    def off(self):
        """ De-power this servo """
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
'''

class Arm(object):
    """
    This class represents an Inmoov Arm
    Arm = wrist + elbow + twist + lift_front + lift_out
    This hierarchy is unlikely to be final
    """

    def __init__(self, hand, wrist, elbow, armtwist, lift_front, lift_out):
        """
        Build an Inmoov Arm
        """
        if hand is None or wrist is None or elbow is None or armtwist is None or lift_front is None or lift_out is None:
            raise Exception("Could not build a arm")
        self.hand = hand
        self.wrist = wrist
        self.elbow = elbow
        self.twist = armtwist
        self.lift_front = lift_front
        self.lift_out = lift_out

    def initialize(self):
        """ set all arm members to default positions in a sequence, with generous waits in between """
        # TODO: might want to change initialization order? order/wait length depends on where the arm position was, and we can't know that
        self.hand.initialize()
        self.wrist.initialize()
        time.sleep(0.5)
        self.elbow.initialize()
        time.sleep(1)
        self.lift_front.initialize()
        self.lift_out.initialize()
        time.sleep(1)
        self.twist.initialize()
        time.sleep(1)

    def off(self):
        """ De-power all members """
        self.hand.off()
        self.wrist.off()
        self.elbow.off()
        self.twist.off()
        self.lift_front.off()
        self.lift_out.off()


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

