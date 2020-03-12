#!/usr/bin/env python

import os.path as op
import io
import time
import sys
import rospy
import std_msgs.msg as rosmsg


write_location = ""

# TODO: revise it to send/receive in binary mode? might be smaller message size, but needs special message type

def callback_onshutdown():
    print("shutdown received")

def callback_newfile(msg):
    # takes a string-type message
    printme = "new file received: ", rospy.get_time(), len(msg.data)
    print(printme)
    rospy.loginfo(printme)
    # write the new file contents to the file location
    # if there is a permission error, keep trying until it succeeds
    while True:
        try:
            with io.open(write_location, "w") as my_file:
                my_file.write(msg.data)
            # if it writes successfully, break
            break
        except PermissionError as pe:
            print(pe)
            rospy.loginfo("permission error, retry in 10sec: " + str(pe))
            time.sleep(10)
        except IOError as ioe:
            print(ioe)
            rospy.loginfo("io error, retry in 10sec: " + str(ioe))
            time.sleep(10)

# usage: rosrun file_transfer file_rcv.py _file:=./whatever.txt
def main():
    global write_location
    # how do i get command-line options into a ros node?
    input_name = None
    try:
        input_name = rospy.get_param('~file')
        # THIS SHOULD WORK, WHY DOESNT THIS WORK!?
    except KeyError:
        # fallback method: sys.argv
        for a in sys.argv:
            if a.startswith("_file:="):
                input_name = a.replace("_file:=", "")
        if input_name is None:
            print("err: no file specified")
            return
    # node/topic name depends on basename of file it is monitoring
    write_location = op.abspath(op.realpath(op.normpath(input_name)))
    if not op.isfile(write_location):
        printme = "input path '%s' clean path '%s' is not a file or does not exist!" % (input_name, write_location)
        print(printme)
        rospy.loginfo(printme)
        return
    basename = op.basename(write_location).replace(" ", "_").replace(".", "_")

    rospy.init_node("/file_transfer/rcv_%s" % basename, anonymous=True)
    rospy.on_shutdown(callback_onshutdown)
    rospy.Subscriber("/file_transfer/%s" % basename, rosmsg.String, callback_newfile)
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException as rosie:
        print(rosie)
        print("oops")
