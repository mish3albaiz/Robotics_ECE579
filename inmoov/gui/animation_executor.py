
from json_parsing import read_json
import time
from os.path import join, dirname
import sys
whereami = dirname(__file__)
scripts_dir= join(whereami, "../scripts/")
sys.path.append(scripts_dir)
import Inmoov

filename_pose = join(whereami, '../json/pose.json')
filename_animation = join(whereami, '../json/animations.json')

# global objects that hold the json file contents
# so i can control when/how often to read the json file
# in the inmoov object, when it receives messages, it only needs to update at bootup. json will not change after bootup.
# in the gui, it should update each time it tries to run, because the gui is editing the files.
global_poses = None
global_animations = None

def update_animations():
    global global_animations
    global_animations = read_json(filename_animation)
def update_poses():
    global global_poses
    global_poses = read_json(filename_pose)

update_animations()
update_poses()

# TODO: if we are keeping the killlist idea, make it cleaner & easy to remove when transferring to a robot that doesn't need it


def do_animation(the_inmoov, animation_name):

    print("Executing animation ", str(animation_name))

    if animation_name not in global_animations:
        print("FAIL TO FIND: ANIMATION '%s'" % str(animation_name))
        return

    #for key, pose_info in sorted(animation_data[animation_name].items()):
    # this method better supports animations >= 10 frames long
    # because using sorted() on 1-12 returns [1, 10, 11, 12, 2, 3, 4, 5, etc]
    this_animation_dict = global_animations[animation_name]
    t = 1
    while str(t) in this_animation_dict:
        # pose_info is a list with item0 = posename and item1 = holdtime
        pose_info = this_animation_dict[str(t)]
        print("\n********* Executing pose {} *********\n".format(str(pose_info[0])))
        do_pose(the_inmoov, pose_info[0], pose_info[1])
        t += 1
    print("\nANIMATION COMPLETE!\n")

#killtime = 1
killlist = ["left_shoulder_lift_front","left_arm_rotate","right_arm_rotate","right_shoulder_lift_front"]

def do_pose(the_inmoov, pose_name, hold_time=0):
    killtime = 1
    if pose_name not in global_poses:
        print("FAIL TO FIND: POSE '%s'" % str(pose_name))
        return
    hold_time = float(hold_time)
    pose_data = global_poses[pose_name]
    for servo_name, servo_angle in pose_data.items():
        #Obtain a handle to the actual servo object
        fservo = the_inmoov.find_servo_by_name(str(servo_name))
        if fservo.curr_angle == servo_angle:
            # if telling it to move to a position it's already at, skip it instead, it doesnt need to move
            print('Skipping', servo_name)
        else:
            fservo.rotate(float(servo_angle))
            print('Setting {} servo to an angle of {}'.format(servo_name, servo_angle))
            if servo_name == 'right_lift_front':
                killtime = abs((7.5/90)*(fservo.curr_angle - servo_angle))

    print('\n--------------- Hold for {} second(s) ---------------'.format(hold_time))

    # todo: handle corner case where hold_time < killtime
    time.sleep(killtime)
    # kill all servos that can safely hold position wihtout power
    for killname in killlist:
        fservo = this_inmoov.find_servo_by_name(str(killname))
        fservo.off()
    
    time.sleep(hold_time - killtime)
    

if __name__ == '__main__':
    this_inmoov = Inmoov.Inmoov()
    
    do_animation(this_inmoov, 'rps_paper')
    time.sleep(5)
    exit()
    do_animation(this_inmoov, 'headright_anim')
    time.sleep(5)
    do_animation(this_inmoov, 'headleft_anim')
    time.sleep(5)
    do_animation(this_inmoov, 'headright_anim')
    time.sleep(5)
