#!/bin/python
"""
This module holds an Inmoov wrist

Authors:
    Brett Creeley
    Matty Baba Allos
"""
from Servo import Servo


class Wrist(object):
    """ This class represents an Inmoov Wrist """

    # TODO: Pull apart Inmoov's forearm to find out servo models for a Wrist. These values are just copied from the HS-805BB Servo.

    def __init__(self, servo):
        """ Set the Servo for this Wrist """

        if servo is None:
            raise Exception("Could not initiate wrist")
        self.servo = servo

    def initialize(self):
        self.servo.initialize()

    def rotate(self, degree):
        """ Rotate this Wrist the desired degree """
        self.servo.rotate(degree)

    def off(self):
        """ Turn off all fingers off"""
        self.servo.off()
