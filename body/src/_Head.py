#!/bin/python
"""
This module holds Inmoov's head

Authors:
    Brett Creeley
    Matty Baba Allos
"""
from Servo import Servo

class Head(object):
    """
    This class is used to control the motion of Inmoov's head.
    - TODO: Add mouth and eye control.
    """

    def __init__(self, x_servo, y_servo):
        """ Initialize all Head Servos and look forward """
        if x_servo is None or y_servo is None:
            raise Exception("Could not initialize Head")
        self.x_servo = x_servo
        self.y_servo = y_servo

    def initialize(self):
        """
        Make head look straight forward x and y-axis
        """
        self.x_servo.initialize()
        self.y_servo.initialize()

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

    def off(self):
        self.x_servo.off()
        self.y_servo.off()
