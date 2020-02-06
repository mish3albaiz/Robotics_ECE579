#!/bin/python
"""
This module contains an Inmoov Arm

Authors:
    Brett Creeley
    Matty Baba Allos
"""
import time

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
