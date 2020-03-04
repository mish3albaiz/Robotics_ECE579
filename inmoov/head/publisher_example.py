#!/usr/bin/env python

import roslib
import rospy
from sensor_msgs.msg import CompressedImage
import io
import time
import picamera
import cv2
import numpy as np


image_pub = None


def capture_image():
    # capture an image and pump it into an "opencv" thing
    
    # Create the in-memory stream
    stream = io.BytesIO()
    with picamera.PiCamera() as camera:
        camera.start_preview()
        time.sleep(2)
        camera.capture(stream, format='jpeg', resize=(360,240))
    # Construct a numpy array from the stream
    data = np.fromstring(stream.getvalue(), dtype=np.uint8)
    # "Decode" the image from the array, preserving colour
    image = cv2.imdecode(data, 1)
    # OpenCV returns an array with data in BGR order. If you want RGB instead
    # use the following...
    # image = image[:, :, ::-1]
    
    return image

def publish_image(image_np):
    #### Create CompressedIamge ####
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
    # Publish new image
    image_pub.publish(msg)

def main():
    global image_pub
    rospy.init_node('inmoov_head')
    image_pub = rospy.Publisher("/output/image_raw/compressed", CompressedImage)
    rate = rospy.Rate(2) # twice per second
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        img = capture_image()
        publish_image(img)
        # taken_image = camera.capture() # not sure if this works
        # pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
