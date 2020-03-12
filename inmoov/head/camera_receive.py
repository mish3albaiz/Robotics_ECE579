#!/usr/bin/env python

import roslib
import rospy
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np




def callback_onimage(msg):
    # takes a string-type message
    logme = "image received: len=%d, time=%s" % (len(msg.data), rospy.get_time())
    rospy.loginfo(logme)
    print(type(msg.data))
    data_np = np.fromstring(msg.data, dtype=np.uint8)
    image = cv2.imdecode(data_np, 1)
    cv2.imwrite("/home/ubuntu/Desktop/receive.jpg", image)
    # note: cv2.imshow doesn't want to cooperate, sadly


def main():
    rospy.init_node('inmoov_head_image_receiver', anonymous=True)
    rospy.loginfo("bootup")
    
    rospy.Subscriber("/inmoov/head/image/compressed", CompressedImage, callback_onimage)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
