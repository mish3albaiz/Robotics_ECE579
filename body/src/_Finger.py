#!/bin/python
"""
This module holds an Inmoov Finger.

Authors:
    Brett Creeley
    Matty Baba Allos
"""
from Servo import Servo

class Finger(object):
    """
    This class represents an Inmoov Finger.
    """

    def __init__(self, servo):
        if servo is None:
            raise Exception("Could not initialize Finger")

        self.servo = servo

    def initialize(self):
        self.servo.initialize()

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

    def off(self):
        self.servo.off()
