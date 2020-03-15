#!/usr/bin/env python

import os.path as op
import io
import sys
import rospy
import std_msgs.msg as rosmsg

file_pub = None
read_location = ""
file_last_changed_time = 0


def file_has_changed():
    # check if the file it is pointing at has changed since it last noticed
    # if it has changed, then update the tracker
    global file_last_changed_time
    newtime = op.getmtime(read_location)
    if newtime > file_last_changed_time:
        file_last_changed_time = newtime
        return True
    return False

def send_file():
    # read + send the file
    try:
        with io.open(read_location, "r") as my_file:
            text = my_file.read()
    # except PermissionError as pe:
    #     print(pe)
    #     rospy.loginfo("permission error: " + str(pe))
    #     return
    except IOError as ioe:
        print(ioe)
        rospy.loginfo("io error: " + str(ioe))
        return
    # "text" is now a string containing the full text of the file
    printme = "sending new file: ", rospy.get_time(), len(text)
    print(printme)
    rospy.loginfo(printme)
    file_pub.publish(text)


# usage: rosrun file_transfer file_xfr.py _file:=./whatever.txt
def main():
    global file_pub
    global read_location
    global file_last_changed_time
    
    rospy.init_node("file_transfer_xfr", anonymous=True)
    # how do i get command-line options into a ros node?
    # NOTE: command-line options must be got AFTER initializing the node! probably, anyway
    input_name = None
    try:
        input_name = rospy.get_param('~file')
        # THIS SHOULD WORK, WHY DOESNT THIS WORK!?
    except KeyError:
        # fallback method: sys.argv
        for a in sys.argv:
            if a.startswith("_file:="):
                print("fallback argv")
                input_name = a.replace("_file:=", "")
        if input_name is None:
            print("err: no file specified")
            return
    
    send_on_launch = None
    try:
        send_on_launch = bool(rospy.get_param('~bootsend'))
        # THIS SHOULD WORK, WHY DOESNT THIS WORK!?
    except KeyError:
        for a in sys.argv:
            if a.startswith("_bootsend:="):
                print("fallback argv")
                send_on_launch = bool(a.replace("_bootsend:=", ""))
        if send_on_launch is None:
            send_on_launch = False

    # topic name depends on basename of file it is monitoring
    read_location = op.abspath(op.realpath(op.normpath(input_name)))
    if not op.isfile(read_location):
        printme = "input path '%s' clean path '%s' is not a file or does not exist!" % (input_name, read_location)
        print(printme)
        rospy.loginfo(printme)
        return
    basename = op.basename(read_location).replace(" ", "_").replace(".", "_")
    
    print("monitoring file:", read_location)
    file_pub = rospy.Publisher("/file_transfer/%s" % basename, rosmsg.String, queue_size=1)
    
    # by default, do not send on launch, overridden by parameters
    if not send_on_launch:
        file_last_changed_time = op.getmtime(read_location)
    
    rate = rospy.Rate(1.0/30.0) # once per 30 seconds at most
    while not rospy.is_shutdown():
        if file_has_changed():
            send_file()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
