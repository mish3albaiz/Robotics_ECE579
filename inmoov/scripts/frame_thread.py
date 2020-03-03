
# Brian Henson

# member of the servo object, launched when servo object is created.
# connects to the servo object, accessses several of its members & functions
# each frame is a sub-pose interpolated between poses.

# exactly 1 thread per servo, reusable. locks exist on pwm object, per-servo frame_queue, per-servo state flags, and per-servo do_set_angle.

# this thread MUST be launched as a "daemon" because this thread should run forever and never actually finish.
# if not a daemon then the python program would never return to the command line until all threads finish, and since this thread runs inside while(true)...
# daemon means "kill this thread when the main thread is terminated" which solves the problem.

# interpolate() actually happens in the main thread but because it is used only for this background thread it makes sense to define it here.


import time


INTERPOLATE_TIME = 0.1

# uses the following members of "servo": 
# running_flag
# idle_flag
# _state_flag_lock
# frame_queue
# _frame_queue_lock
# framethread (debugging)
# id (debugging)
# do_set_angle()
def Frame_Thread_Func(servo, DEBUG_SERVO_ID):
    # looping forever
    while True:
    
        # wait until servo."running" event is set by servo object
        servo.running_flag.wait()

        if servo.id == DEBUG_SERVO_ID:
            print("%s: thread wakeup" % servo.framethread_name)

        while True:
            frame = None
            # if there are frames in the frame queue, pop one off (with lock). otherwise, break.
            # if an abort happened while sleeping, the queue will be empty and it will exit, no separate event needed.
            with servo._frame_queue_lock:
                if len(servo.frame_queue) > 0:
                    frame = servo.frame_queue.pop(0)
            if frame is None:   # "else" but outside of the lock block
                break
                
            if servo.id == DEBUG_SERVO_ID:
                print("%s: execute frame %s" % (servo.framethread_name, frame))

            # set the servo to the pose indicated by the frame
            # skip the safety checks in set_servo_angle or set_leg_position because safety is checked before interpolating
            # use the unprotected servo member function: also updates the position stored in the servo
            servo.do_set_servo_angle(frame[0])
            
            # sleep for frame-delay
            time.sleep(frame[1])
            pass
            
        # now frame queue is empty!
        if servo.id == DEBUG_SERVO_ID:
            print("%s: thread sleep" % servo.framethread_name)
            
        with servo._state_flag_lock:
            # clear "running" event, does not trigger anything (note: clear before set)
            servo.running_flag.clear()
            # set the "sleeping" event, this may trigger other waiting tasks
            servo.idle_flag.set()
        # loop back to top, wait until running_flag is set again
        pass
    pass



# return a list of lists
# interpolating in angle space, so its all floating-point inputs/outputs
# time between interpolated poses is very close to INTERPOLATE_TIME but not exactly
# num_frames * frame_time exactly equals requested duration
# minimum duration is INTERPOLATE_TIME, rounds up if it is less
def interpolate(dest, curr, dur):
    # find the delta(s)
    delta = dest - curr
    # total time must not be less than INTERPOLATE_TIME
    dur = max(dur, INTERPOLATE_TIME)
    # determine how many sections this time must be broken into: round up/down to nearest whole number
    num_frames = round(dur / INTERPOLATE_TIME)
    # from number of frames, calculate the time between each frame (should be very close to INTERPOLATE_TIME)
    frame_time = dur / num_frames
    # build the list with list comprehension trickery
    # math is ((i+1)/num_frames * delta) + curr
    # the +1 is because the first frame should NOT be curr, and the final frame SHOULD be dest (curr+delta)
    frame_list = [[((i+1)/num_frames * delta) + curr, frame_time] for i in range(num_frames)]
    # done building the frame-list
    return frame_list

