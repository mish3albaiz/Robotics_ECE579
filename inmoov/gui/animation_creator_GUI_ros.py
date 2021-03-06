#!/usr/bin/env python


import sys
import animation_creator_GUI
import rospy
import std_msgs.msg as rosmsg



def send_with_ros(message):
    # i'm concerned that this might be sending too many messages over ROS, but oh well, we wont' know until we try
    if not rospy.is_shutdown():
        # print(message)
        rospy.loginfo(message)
        pub.publish(message)
    
    
    
if __name__ == '__main__':
    try:
        pub = rospy.Publisher('/inmoov/body/poser', rosmsg.String, queue_size=10)
        rospy.init_node('inmoov_animation_gui', anonymous=True)
        # launch the gui but give it the callback that sends over ROS instead of locally
        animation_creator_GUI.launch_gui(send_with_ros)
    except rospy.ROSInterruptException:
        print("oops")
        


