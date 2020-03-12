#!/usr/bin/env python

import sys
print(sys.version)

import rospy
import std_msgs.msg as rosmsg
from Inmoov import Inmoov



# set up the body and also set up ROS, probably

# pub = rospy.Publisher('topic_name', rosmsg.String, queue_size=10)
# pub.publish(rosmsg.String("foo"))


# msg = std_msgs.msg.String("hello world")
# msg = std_msgs.msg.ColorRGBA(255.0, 255.0, 255.0, 128.0)



inMoov = Inmoov()

def callback_onshutdown():
    print("shutdown received")
    inMoov.off()

def callback_onmessage_oneservocontrol(msg):
    # takes a string-type message
    print("message received: ", rospy.get_time(), msg.data)
    rospy.loginfo(msg.data)
    inMoov.inmoov_ros_command(msg.data)
    
def main():
    # logging channel?
    rospy.init_node("inmoov_body_bodyposer_receive", anonymous=True)
    rospy.on_shutdown(callback_onshutdown)
    rospy.Subscriber("/inmoov/body/poser", rosmsg.String, callback_onmessage_oneservocontrol)
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException as rosie:
        print(rosie)
        pass
