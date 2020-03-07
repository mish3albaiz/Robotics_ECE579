
from json_parsing import read_json
import time
from os.path import join, dirname
import sys
whereami = dirname(__file__)
scripts_dir= join(whereami, "../scripts/")
sys.path.append(scripts_dir)
from Inmoov import *

this_inmoov = Inmoov()

filename_pose = join(whereami, '../json/pose.json')
filename_animation = join(whereami, '../json/animations.json')


# TODO: this function is a member of the inmoov object

def execute_animation(animation_id):
    
    #Open up the JSON and get the high level dictionary containing a list of poses to execute
    animation_data = {}
    animation_data = read_json(filename_animation)
    
    print("Executing animation ", str(animation_id))
    
    #Top level dict keys: pose ID
    #Top level value: Another dictionary of servo IDS and PWMs
    for key, pose_info in sorted(animation_data[animation_id].items()):

        pose_name = pose_info[0]
        
        print("\n********* Executing pose {} *********\n".format(str(pose_name)))
            
        #Int value defining time to hold the pose
        sleep_time = pose_info[1]
            
        #Open up the file containing for the pose data
        pose_file_data = read_json(filename_pose)
            
        pose_data = {}
        pose_data = pose_file_data[pose_name]
            
        for servo_id, servo_angle in pose_data.items():
            #Obtain a handle to the actual servo object
##            servo = self.find_servo_by_name(str(servo_id))
##            servo.rotate(servo_angle)

            print('Setting {} servo to an angle of {}'.format(servo_id, servo_angle))

        print('\n--------------- Hold for {} second(s) ---------------'.format(sleep_time))
        time.sleep(int(sleep_time))
        
    
    
##	return

def do_animation(animation_id):
    animation_data = {}
    animation_data = read_json(filename_animation)

    print("Executing animation ", str(animation_id))

    #for key, pose_info in sorted(animation_data[animation_id].items()):
    # this method better supports animations >= 10 frames long
    # because using sorted() on 1-12 returns [1, 10, 11, 12, 2, 3, 4, 5, etc]
    this_animation_dict = animation_data[animation_id]
    t = 1
    while str(t) in this_animation_dict:
        pose_info = this_animation_dict[str(t)]
        print("\n********* Executing pose {} *********\n".format(str(pose_info[0])))
        do_pose(pose_info[0], pose_info[1])
        t += 1
    print("\nANIMATION COMPLETE!\n")

killtime = 1
killlist = ["left_shoulder_lift_front","left_arm_rotate","right_arm_rotate","right_shoulder_lift_front"]

def do_pose(pose_name, hold_time):
    hold_time = float(hold_time)
    pose_file_data = read_json(filename_pose)
    pose_data = {}
    pose_data = pose_file_data[pose_name]
    for servo_id, servo_angle in pose_data.items():
        #Obtain a handle to the actual servo object
        fservo = this_inmoov.find_servo_by_name(str(servo_id))
        if fservo.curr_angle != servo_angle:
            fservo.rotate(int(servo_angle))
            print('Setting {} servo to an angle of {}'.format(servo_id, servo_angle))
        else:
            print('Skipping', servo_id)
            
    print('\n--------------- Hold for {} second(s) ---------------'.format(hold_time))

    # todo: handle corner case where hold_time < killtime
    time.sleep(killtime)
    # kill all servos that can safely hold position wihtout power
    for killname in killlist:
        fservo = this_inmoov.find_servo_by_name(str(killname))
        fservo.off()
    
    time.sleep(hold_time - killtime)
    
do_animation('rps')
time.sleep(5)
exit()
do_animation('headright_anim')
time.sleep(5)
do_animation('headleft_anim')
time.sleep(5)
do_animation('headright_anim')
time.sleep(5)
