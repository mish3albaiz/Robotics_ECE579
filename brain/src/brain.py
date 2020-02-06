#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16
from sensor_msgs.msg import CompressedImage
from utility import *

import sys
#import OpenCV module
import cv2
#import os module for reading training data directories and paths
import os
#import numpy to convert python lists to numpy arrays as 
#it is needed by OpenCV face recognizers
import numpy as np
import datetime

#from scipy.ndimage import filters

#from imutils.video import VideoStream
#from imutils.video import FPS
import argparse
import imutils
import time


subjects = ["", "Matty Baba Allos", "Max Schweitzer"]

count = 0;
person = 0;
previous_person = 0;
sent = 0;
new = 0;
blip = 0;

face_recognizer = cv2.face.LBPHFaceRecognizer_create()
print("Read trained model")
face_recognizer.read('trained.xml')

def predict(test_img):
    #make a copy of the image as we don't want to chang original image
    global person
    global previous_person
    global face_recognizer

    np_arr = np.fromstring(test_img, np.uint8)
    img_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    img = img_np.copy()
    #detect face from the image
    face, rect = detect_face(img)

    #predict the image using our face recognizer 
    label, confidence = face_recognizer.predict(face)
    #get name of respective label returned by face recognizer
    if (rect[0] == 0 and rect[1] == 0 and rect[2] == 0 and rect[3] == 0):
      label = 0
    label_text = subjects[label]
    #print("Label %d", label)
    #print("Confidence %d", confidence)
    
    #draw a rectangle around face detected
    draw_rectangle(img, rect)
    #draw name of predicted person
    draw_text(img, label_text, rect[0], rect[1]-5)
    
    #print("label_text ", label_text)
    #print("label ", label)
    if confidence < 40:
      #print("test")
      return img, "none", 0

    return img, label_text, label

# Assign specific motions to specific people.
def action(person):
    if person == "Max Schweitzer":
      action = 0
    elif person == "Matty Baba Allos":
      action = 1
    elif person == "Armaan Roshani":
      action = 2
    elif person == "Chad Coates":
      action = 3
    elif person == "Kestutis Sultanas":
      action = 4
    elif person == "none":
      action = 99
    else:
      action = 99
    return action


class brain:
   def __init__(self):
       # Subscribe to the picam node to get images, also publish to the motion topic, to send motion information.
       self.pub = rospy.Publisher('motion', Int16, queue_size=10)
       self.subscriber = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.callback,  queue_size =1)


   def callback(self,msg):
       global person
       global previous_person
       global count
       global sent
       global new
       global blip
       rospy.loginfo("got an image")
       #call openCV work here
       previous_person = person
       frame_predict, person, label = predict(msg.data)

       # draw the timestamp on the frame
       timestamp = datetime.datetime.now()
       ts = timestamp.strftime("%A %d %B %Y %I:%M:%S%p")
       cv2.putText(frame_predict, ts, (10, frame_predict.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)

       # show the frame
       cv2.imshow('Feed', frame_predict)
       key = cv2.waitKey(1) & 0xFF

       if (person == previous_person):
         count += 1

       if (person != previous_person):
         count = 0
         new = not new


       if (((new == 0) and (count >= 20))):
         result = action(person)
         print(result)
         count = 0
         new = 1
         self.pub.publish(result)


def main():
   rospy.init_node('brain', anonymous=True)
   b =brain()

   try:
       rospy.spin()
   except KeyboardInterrupt:
       rospy.loginfo("Inmoov brain is dead")

if __name__ == '__main__':
 main()

