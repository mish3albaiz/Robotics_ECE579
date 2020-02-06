#!/bin/python
"""
This module holds Inmoov's Torso

Authors:
    Brett Creeley
    Matty Baba Allos
"""
from Servo import Servo


class Torso(object):
    """ This class is used to control Inmoov's Torso """

    def __init__(self, left_servo, right_servo):
        """
        Initialize all of Inmoov's Torso variables.
        """
        if left_servo is None or right_servo is None:
            raise Exception("Could not initialize Torso")
        self.l_servo = left_servo
        self.r_servo = right_servo

    def initialize(self):
        self.l_servo.initialize()
        self.r_servo.initialize()

    def lean(self, degrees):
        """
        Make Inmoov lean based on the specified degree.
        - -90 degrees leans Inmoov all the way right.
        -   0 degrees centers Inmoov's Torso.
        -  90 degrees leans Inmoov all the way left.
        """
        self.l_servo.rotate(degrees)
        self.r_servo.rotate(degrees)

    def off(self):
        """ Turn off all fingers off"""
        self.l_servo.off()
        self.r_servo.off()
