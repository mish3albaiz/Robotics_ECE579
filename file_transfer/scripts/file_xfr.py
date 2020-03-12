#!/usr/bin/env python

import os.path as op
import io
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
    except PermissionError as pe:
        print(pe)
        rospy.loginfo("permission error: " + str(pe))
        return
    except IOError as ioe:
        print(ioe)
        rospy.loginfo("io error: " + str(ioe))
        return
    # "text" is now a string containing the full text of the file
    rospy.loginfo("sending new file: ", rospy.get_time(), len(text))
    file_pub.publish(text)


# usage: rosrun file_transfer file_xfr.py _file:=./whatever.txt
def main():
    global file_pub
    global read_location
    global file_last_changed_time
    
    # how do i get command-line options into a ros node?
    input_name = "a/b/c/d/"
    try:
        input_name = rospy.get_param('~file')
    except KeyError:
        print("err: no file specified")
        return
    
    try:
        send_on_launch = bool(rospy.get_param('~bootsend'))
    except KeyError:
        send_on_launch = False

    # node/topic name depends on basename of file it is monitoring
    read_location = op.abspath(op.realpath(op.normpath(input_name)))
    if not op.isfile(read_location):
        rospy.loginfo("input path '%s' clean path '%s' is not a file or does not exist!" % (input_name, read_location))
        return
    basename = op.basename(read_location).replace(" ", "_").replace(".", "_")
    
    rospy.init_node("/file_transfer/xfr_%s" % basename, anonymous=True)
    file_pub = rospy.Publisher("/file_transfer/%s" % basename, rosmsg.String)
    
    # by default, do not send on launch, overridden by parameters
    if not send_on_launch:
        file_last_changed_time = op.getmtime(read_location)
    
    rate = rospy.Rate(1.0/30.0) # once per 30 seconds at most
    # will always send once on bootup
    while not rospy.is_shutdown():
        if file_has_changed():
            send_file()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass