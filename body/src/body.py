#!/usr/bin/env python
import rospy



# set up the body and also set up ROS, probably



from std_msgs.msg import Int16

from Inmoov import Inmoov
inMoov = Inmoov()

def callback(msg):
    rospy.loginfo(msg.data)
    inMoov.do_motion(msg.data)

def main():
    rospy.init_node("body")
    rospy.Subscriber("/motion", Int16, callback)
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
