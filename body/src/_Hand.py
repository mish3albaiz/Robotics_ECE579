#!/bin/python
"""
This module holds an Inmoov Hand.

Authors:
    Brett Creeley
    Matty Baba Allos
"""
from _Finger import Finger


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


    def initialize(self):
        self.pinky_finger.initialize()
        self.ring_finger.initialize()
        self.mid_finger.initialize()
        self.index_finger.initialize()
        self.thumb.initialize()


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

    def off(self):
        """ Turn off all fingers off"""
        self.pinky_finger.off()
        self.ring_finger.off()
        self.mid_finger.off()
        self.index_finger.off()
        self.thumb.off()
