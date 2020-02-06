#!/bin/python
"""
This module contains an Inmoov Shoulder

Authors:
    Brett Creeley
    Matty Baba Allos
"""
from Servo import Servo


class Shoulder(object):
    """ This class represents an Inmoov Shoulder """

    def __init__(self, flexion_servo, abduction_servo, rotation_servo_x,rotation_servo_y ):
        """ Build an Inmoov Shoulder """
        if flexion_servo is None or abduction_servo is None or rotation_servo_x is None or rotation_servo_y is None:
            raise Exception("Could not initiate shoulder")
        self.flexion_servo = flexion_servo
        self.abduction_servo = abduction_servo
        self.rotation_servo_x = rotation_servo_x
        self.rotation_servo_y = rotation_servo_y

    def initialize(self):
        """
        Make the arm in down postion
        """
        self.flexion_servo.initialize()
        self.abduction_servo.initialize()
        self.rotation_servo_x.initialize()
        self.rotation_servo_y.initialize()

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

    def off(self):
        self.rotation_servo_x.off()
        self.rotation_servo_y.off()
        self.abduction_servo.off()
        self.flexion_servo.off()
