#!/usr/bin/env python
try:
    import Tkinter as tk
    import tkFont as tkFont
except ModuleNotFoundError:
    import tkinter as tk
    from tkinter import font as tkFont
import sys
from os.path import join, dirname
whereami = dirname(__file__)
scriptsdir = join(whereami, "../scripts/")
sys.path.append(scriptsdir)
import Inmoov
import json_parsing as jp

# create the INMOOV as a lazy way to parse the JSON and stuff
# or when running locally, this actually instantiates & controls the servos
my_inmoov = Inmoov.Inmoov()

import time

filename_pose = join(whereami, '../json/pose.json')
filename_animation = join(whereami, '../json/animations.json')

button_padx = 5
button_pady = 5

class Application(tk.Frame):
    def __init__(self, master, do_pose_callback):
        tk.Frame.__init__(self, master)

        self.do_pose_callback = do_pose_callback

        self.animation_frame = tk.Frame(master,borderwidth=2, relief="solid")
        self.animation_frame.grid(row=1, column =0,padx = 5,pady = 5, sticky = 'wens')
        self.pose_frame = tk.Frame(master, borderwidth=2, relief="solid")
        self.pose_frame.grid(row = 1, column = 1,padx = 5,pady = 5,sticky = 'swne')
        self.button_frame = tk.Frame(self.animation_frame)
        self.button_frame.grid(row = 6, column =0,padx = 5, pady = 10, sticky = 'we', columnspan = 3)
        self.pose_button_frame = tk.Frame(self.animation_frame, borderwidth=1, relief="solid")
        self.pose_button_frame.grid(row = 7, column =0,padx = 5, pady = 10, sticky = 'we', columnspan = 3, ipadx  =5, ipady = 5)

        self.on_off_load = False

        # dictionary to hold all the poses in an animation
        self.PoseDict = {}
        # list to hold all possible pose options
        self.OptionList = []

        self.AniList = []

        # read and save data in pose.json file
        self.data = jp.read_json(filename_pose)

        self.ani_data = jp.read_json(filename_animation)

        # where to start displaying pose information
        self.current_pose = 1

        # gather all possible pose values and store in list
        for pose in self.data:
            self.OptionList.append(pose)

        for animation in self.ani_data:
            self.AniList.append(animation)

        # making dynamic variable to change when pose option menu changes
        self.variable = tk.StringVar(master)

        self.animation = tk.StringVar(master)

        # setting pose option menu to initially be the first pose in the JSON file
        self.variable.set(self.OptionList[0])

        # setting animation load option
        self.animation.set(self.AniList[0])

        # widgets in main GUI layout
        
        tk.Label(self.animation_frame, text = 'Enter Animation Name In Entry Box and Click Save Button').grid(row = 0, column = 0, columnspan = 3, sticky = 'we',padx = button_padx)

        self.entry_save = tk.Entry(self.animation_frame)
        self.entry_save.grid(row = 1, column =0, columnspan = 2, sticky = 'we',padx = button_padx)

        self.button_save = tk.Button(self.animation_frame, text = 'SAVE', command = lambda: self.save_animation())
        self.button_save.grid(row = 1, column = 2,padx = button_padx, sticky = 'we')

        tk.Label(self.animation_frame, text = '----------------------------------------------------------').grid(row = 2, column = 0, columnspan = 3, sticky = 'we',padx = button_padx)

        self.pose_label = tk.Label(self.animation_frame, text = 'Select Pose, Enter Hold Time (s), Click Add Button')
        self.pose_label.grid(row = 3, column = 0, columnspan = 3, sticky = 'we',padx = button_padx)
        
        self.opt = tk.OptionMenu(self.animation_frame, self.variable, *self.OptionList, command = lambda x = self.variable.get(): self.view_pose(x))
        self.opt.grid(row=4, column=0,padx = button_padx, sticky = 'we')

        self.entry_time = tk.Entry(self.animation_frame)
        self.entry_time.grid(row = 4, column = 1)

        self.button_add = tk.Button(self.animation_frame, text = 'APPEND', command = lambda: self.add_pose())
        self.button_add.grid(row = 4, column = 2, sticky = 'we', padx = 5)

        self.listbox_widget = tk.Listbox(self.animation_frame)
        self.listbox_widget.grid(row = 5, columnspan = 3, sticky = 'wens', padx = 3, pady = 3)

        self.reset_button = tk.Button(self.button_frame, text = 'CLEAR', command = lambda: self.clear_animation(),bg = 'firebrick1')
        self.reset_button.grid(row = 0, column = 0, sticky = 'we', padx = 8)

        self.execute_button = tk.Button(self.button_frame, text = 'EXECUTE', command = lambda: self.execute_poses(),bg = 'SpringGreen2')
        self.execute_button.grid(row = 0, column = 1, sticky = 'we', padx = 8)

        self.load_button = tk.Button(self.button_frame, text = 'LOAD', bg = 'deep sky blue', command = lambda:self.show_load_animation())
        self.load_button.grid(row = 0, column = 2, sticky = 'we', padx = 8)

        tk.Label(self.pose_button_frame, text = 'Select a Pose from the List Above to Use Buttons Below').grid(row = 0, column = 0, columnspan = 3, sticky = 'we',padx = button_padx)

        self.edit_button = tk.Button(self.pose_button_frame, text = 'EDIT TIME', bg = 'gray77', command = lambda:self.edit_time_view())
        self.edit_button.grid(row = 1, column = 0, sticky = 'we', padx = 8)

        self.position_button = tk.Button(self.pose_button_frame, text = 'EDIT ORDER', bg = 'gray77', command = lambda: self.edit_order())
        self.position_button.grid(row = 1, column = 1, sticky = 'we', padx = 8)

        self.delete_button = tk.Button(self.pose_button_frame, text = 'DELETE', bg = 'gray77', command = lambda: self.delete_pose())
        self.delete_button.grid(row = 1, column = 2, sticky = 'we', padx = 8)

        self.ani_select = tk.OptionMenu(self.button_frame, self.animation, *self.AniList, command = lambda x = self.animation.get():self.set_animation(x))
        self.ani_select.config(bg = 'gray77')
        self.ani_select.grid(row=1, column=0, columnspan = 3,padx = 5, sticky = 'we', pady = 10)
        self.ani_select.grid_remove()

        self.animation_confirm = tk.Button(self.button_frame, text = 'OPEN', command = lambda: self.load_animation(), width = 10, bg = 'gray77')
        self.animation_confirm.grid(row = 1, column = 3,padx = 5, sticky = 'we', pady = 10)
        self.animation_confirm.grid_remove()

        # defining grid weights for various frames
        self.button_frame.grid_columnconfigure(0, weight=1)
        self.button_frame.grid_columnconfigure(1, weight=1)
        self.button_frame.grid_columnconfigure(2, weight=1)
        self.pose_button_frame.grid_columnconfigure(0, weight=1)
        self.pose_button_frame.grid_columnconfigure(1, weight=1)
        self.pose_button_frame.grid_columnconfigure(2, weight=1)

        tk.Label(self.pose_frame, text = 'Scroll to view selected pose values:').grid(row = 0, column = 0, sticky = 'w',padx = button_padx)
        
        self.pose_listbox = tk.Listbox(self.pose_frame, width=40, height=26)
        self.pose_listbox.grid(row = 1,column = 0, sticky = 'nswe', padx = 5)

        # view the pose selected by option menu
        self.view_pose(self.variable.get())

    # add_pose: adds a pose to the current animation
    def add_pose(self):
        try:
            # obtain hold time from entry box
            hold_time = float(self.entry_time.get())
            # confirm time is greater than 0
            if hold_time > 0:
                self.PoseDict[(self.current_pose)] = [self.variable.get(), hold_time]
                # view animation in GUI
                self.view_animation()
                # clear entry text
                self.entry_time.delete(0, 'end')
				# updating class pose number
                self.current_pose = self.current_pose + 1
            # create error if invalid hold time is provided
            else:
                # display error message
                self.display_error('add_error')
                # clear entry text
                self.entry_time.delete(0, 'end')
        except:
	    # display error message
            self.display_error('add_error')
	    # clear the hold time entry box
            self.entry_time.delete(0, 'end')

    # save_pose: save the current animation to animation JSON file
    def save_animation(self):
	# check to make sure entry box is not empty
        if len(self.entry_save.get()) != 0:
	    # add new object to JSON woth enrty box name as key and PoseDict as the values
            jp.add_object_to_json(filename_animation, self.entry_save.get(), self.PoseDict)
	    # indicate that the pose has been saved
            print(self.entry_save.get() + 'animation has been saved.')
        else:
	    # display error indicating that a save name must be provided
            self.display_error('save_error')

    # display_error: creates popup window based in the type of error
    #   error_type (string) - indicates what type of error has happened
    def display_error(self, error_type):
		# create popup window
        toplevel = tk.Toplevel()
		# title popup window "ERROR"
        toplevel.title("ERROR")
		# if the error deals with adding a pose
        if error_type == 'add_error':
	    # indicate that a numical value must be given to add a pose to the animation
            display_string = 'Please provide a numerical value for pose hold time.'
        # of the error deals with saving an animation
        else:
	    # indicate that a name must be given in order to save an animation
            display_string = 'Please provide an animation name.'
		
        # label saying ERROR! colored red
        tk.Label(toplevel, text = "ERROR!", fg = 'red').grid(row=0, column=0, sticky = 'we')
	# displaying error message depending on type of error
        tk.Label(toplevel, text = display_string).grid(row = 1, column = 0)
	# indicating that the window should be exited in order to change other values
        tk.Label(toplevel, text = "Click 'x' in top right corner to exit this window.").grid(row = 2, column = 0)

    # view_pose: displays currently selected pose from option menu on right hand part of GUI
    #   pose_name (string) - the pose that needs to be displayed
    def view_pose(self, pose_name):
	# clear the list box containing pose information
        self.pose_listbox.delete(0,tk.END)
	# get the font of the list box
        listFont = tkFont.Font(font=self.pose_listbox.cget("font"))
	# determine how large a space is in the list box font
        spaceLength = listFont.measure(" ")
	# set a perferred gap between servo name and servo angle value
        spacing = 30 * spaceLength
	# read in poses information from JSON file 
        pose = jp.read_json(filename_pose)
	# create empty list for servo values
        servo_list = []
	# create empty list for pwm/angle values
        pwm_list = []
        
	# iterate through pose object within JSON information
        for servo, pwm_value in pose[self.variable.get()].items():
	    # get servo name and append to servo list
            servo_list.append(servo)
            # get servo pwm and append to pwm list
            pwm_list.append(pwm_value)
		
	# determine of long each servo name is
        leftLengths = [listFont.measure(s) for s in servo_list]
	# get the longest length of a servo name
        longestLength = max(leftLengths)
		
	## NOTE: for some reason the spacing is not consistent when using the PI but is when using a Windows PC
		
	# for the values in servo list
        for i in range(len(servo_list)):
	    # if the value has an even index color the text gray
            if i%2 == 0:
                color = 'gray95'
            # if the value has an odd index color the text white
            else:
                color = 'white'
            #figure out spacing to format each line to have the same length
            neededSpacing = longestLength + spacing - leftLengths[i]
	    # how many spaces need to be added in order to keep each line the same length
            spacesToAdd = int(round(neededSpacing/spaceLength))
	    # add pose servo and pwm value to list box
            self.pose_listbox.insert(tk.END, "  " + servo_list[i] + spacesToAdd*" " + str(pwm_list[i]))
	    # set color or list box text entry
            self.pose_listbox.itemconfig(i, {'bg':color})

    # view_animation: shows animation using listbox widget on left side of screen
    def view_animation(self):
	# create empty pose list
        pose_list = []
	# clear list box holding animation information
        self.listbox_widget.delete(0,tk.END)
	# reset i to 1
        i = 1
	# iterate through PoseDict
        for pose, pose_info in self.PoseDict.items():
            # construct string that will displayed in animation list bot
            pose_string = "{}. Hold {} pose for {} second(s)".format(i, pose_info[0], pose_info[1])
            # append string to pose list
            pose_list.append(pose_string)
            # increment i
            i = i + 1
        # iterate through pose_list
        for entry in pose_list:
            # add string to text box
            self.listbox_widget.insert(tk.END, entry)

    # clear_animation: clears contents of PoseDict, which holds the information for the current animation
    def clear_animation(self):
        # clear PoseDict
        self.PoseDict = {}
        # clear animation list box
        self.listbox_widget.delete(0,tk.END)
        # reset current pose number
        self.current_pose = 1
        # update animation list box (to be empty)
        self.view_animation()

    # execute_pose: executes all poses within PoseDict
    def execute_poses(self):
        # iterate through a sorted PiseDict
        for pose_id, pose_info in sorted(self.PoseDict.items()):
            # indicate that is starting to execute a pose
            print("\n********* Executing pose {} *********\n".format(str(pose_info[0])))
            # execute pose
            self.do_pose_callback("pose!" + str(pose_info[0]))
            # wait for the predetermined hold time
            time.sleep(float(pose_info[1]))
        # indicate that the animation has completed
        print("\nANIMATION COMPLETE!\n")

    # show_load_animation: shows/hides animation load functionality
    def show_load_animation(self):
        # flip the state of the on/off variable
        self.on_off_load = not self.on_off_load
        # if items should be shown
        if self.on_off_load:
            # show animation selector
            self.ani_select.grid(row=1, column=0, columnspan = 2,padx = 5, sticky = 'we', pady = (20,0))
            # show load button
            self.animation_confirm.grid(row = 1, column = 2,padx = 5, sticky = 'we', pady = (20, 0))
        # if items should be hidden
        else:
            # hide animation selector
            self.ani_select.grid_remove()
            # hide load button
            self.animation_confirm.grid_remove()

    # edit_time_view: displays popup window for editing the hold time of a pose     
    def edit_time_view(self):
        # try to obtain a selection from animation listbox
        try:
            current_pose_selection = int(self.listbox_widget.curselection()[0]) + 1
        # if no item is slected within listbox then return
        except:
            return
        # get the information stored in PoseDict at selected key value index
        info = self.PoseDict.get(current_pose_selection)
        # clear list box selection
        self.listbox_widget.selection_clear(0, tk.END)
        # create popup window
        self.toplevel = tk.Toplevel()
        # title popup window
        self.toplevel.title("Change Pose Duration")
        # setting all the columns in the popup window to be the same
        self.toplevel.grid_columnconfigure(0, weight=1)
        self.toplevel.grid_columnconfigure(1, weight=1)
        self.toplevel.grid_columnconfigure(2, weight=1)
        # format string to indicate what the selected pose hold time currently is
        string1 = "Current Time Duration for {} pose is {} second(s).".format(info[0], info[1])
        # display formatted string above
        tk.Label(self.toplevel, text = string1).grid(row=0, column=0, columnspan = 3)
        # label indicating the change time entry box
        tk.Label(self.toplevel, text = "Change to:").grid(row = 1 , column = 0)
        # entry box for entering the desired new hold time
        self.time_entry = tk.Entry(self.toplevel)
        self.time_entry.grid(row = 1, column = 1)
        # confirmation button
        tk.Button(self.toplevel, text = "OK", command = lambda: self.change_time(current_pose_selection, self.time_entry.get())).grid(row = 1, column = 2)
        # reminding user to close window before changing other values
        tk.Label(self.toplevel, text = "Remember to close this window before changing any other time value").grid(row=2, column=0, columnspan = 3)

    # load_animation: loads an animation from the JSON file and populates PoseDict
    def load_animation(self):
        # clear PoseDict
        self.PoseDict = {}
        # reset i value
        i = 1
        # iterate through PoseDict
        for pose_num in sorted(self.ani_data[self.animation.get()]):
            # obtain and save pose information from JSON
            self.PoseDict[i] = self.ani_data[self.animation.get()][str(i)]
            # increment i
            i = i + 1
        # set current pose number
        self.current_pose = i
        # view animation in list box
        self.view_animation()
        # clear save entry box
        self.entry_save.delete(0, 'end')
        # load animation name into save entry box
        self.entry_save.insert(0,self.animation.get())
        # show load animation functionality
        self.on_off_load  = True
        self.show_load_animation()

    # set_animation: sets the class animation variable to that of an option menu
    #   value (string) - indicates the desired animaiton
    def set_animation(self, value):
        # get animation from list
        index_value = self.AniList.index(value)
        # set current animation
        self.animation.set(self.AniList[index_value])

    # change_time: changes the hold time of a pose within PoseDict
    #   pose_index (integer) - index of pose that is being changed
    #   new_time (float) - new hold time for selected pose
    def change_time(self, pose_index, new_time):
        # get current pose hold time
        self.PoseDict[pose_index][1] = float(new_time)
        # close popup window
        self.toplevel.destroy()
        # update list box with new hold time
        self.view_animation()

    # delete_pose: deletes a selected pose from PoseDict
    def delete_pose(self):
        # try to obtain selected value
        try:
            # get index vlaue of current selection
            current_pose_selection = int(self.listbox_widget.curselection()[0]) + 1
        # return if no item is selected
        except:
            return
        # create empty lists for keys and vlaues of PoseDict
        key_list = []
        value_list = []
        # get pose information from PoseDict
        info = self.PoseDict.get(current_pose_selection)
        # deselect item in list box
        self.listbox_widget.selection_clear(0, tk.END)

        # obtain all values from PoseDict
        for key, value in sorted(self.PoseDict.items()):
            value_list.append(value)

        # delete pose at desired location
        del value_list[current_pose_selection-1]
        # clear PoseDict
        self.PoseDict = {}

        # repopulate PoseDict so now deleted item is not included
        i = 1
        for x in value_list:
            self.PoseDict[i] = x
            i = i+1

        # update list box widget with updated animation
        self.view_animation()

    # edit_order: creates popup window to change pose order
    def edit_order(self):
        # try to obtain slection from animation list box
        try:
            current_pose_selection = int(self.listbox_widget.curselection()[0]) + 1
        # do nothing and return if no selection has been made
        except:
            return
        # get selected pose information
        info = self.PoseDict.get(current_pose_selection)
        # clear list box selection
        self.listbox_widget.selection_clear(0, tk.END)
        # create popup window
        self.toplevel = tk.Toplevel()
        # title popup window
        self.toplevel.title("Change Pose Position")
        # set all columns in popup window to have same weight
        self.toplevel.grid_columnconfigure(0, weight=1)
        self.toplevel.grid_columnconfigure(1, weight=1)
        self.toplevel.grid_columnconfigure(2, weight=1)
        # format string indicating the current position of the selected animation
        string1 = "{} is currently in position {} in the animation.".format(info[0], current_pose_selection)
        # display formated string in popup window
        tk.Label(self.toplevel, text = string1).grid(row=0, column=0, columnspan = 3)
        tk.Label(self.toplevel, text = "Change to:").grid(row = 1 , column = 0)
        # entry to enter new pose position number
        self.number_entry = tk.Entry(self.toplevel)
        self.number_entry.grid(row = 1, column = 1)
        # confirm button used to change position number
        tk.Button(self.toplevel, text = "OK", command = lambda: self.reorder_poses(int(self.number_entry.get()), current_pose_selection)).grid(row = 1, column = 2)
        tk.Label(self.toplevel, text = "Remember to close this window before changing any other position value").grid(row=2, column=0, columnspan = 3)

    # reorder_poses: reorders the poses in PoseDict
    #   old_position (integer) - the position that the pose has been at
    #   new_position (integer) - the position that the pose needs to be at
    def reorder_poses(self, new_position, old_position):
        # unify JSON and pose numbering to list indexes
        new_position = new_position -1
        old_position = old_position - 1
        # create empty lists to aid in resorting 
        value_list = []
        bottom_list = []
        top_list = []
        save_list = []

        # sort PoseDict keys smallest to largest
        for key, value in sorted(self.PoseDict.items()):
            # append vlaues to list
            value_list.append(value)

        # saving the old positions info
        save_list.append(value_list[old_position])

        # check to make sure position is valid
        if new_position not in range(0, len(self.PoseDict)):
            return
        # if pose gets moved forward in animation
        elif new_position < old_position:
            # obtain values below new position
            bottom_list = value_list[:(new_position)]
            # obtain values above new position
            top_list = value_list[new_position:]
            # remove old duplicate value from top list
            top_list.remove(save_list[0])
        # if pose gets moved back in animation
        elif new_position > old_position:
            # obtain values below new position
            bottom_list = value_list[:(new_position+1)]
            # obtain values above new position
            top_list = value_list[new_position+1:]
            # delete duplicate in bottom list
            bottom_list.remove(save_list[0])
        # if position is the same or doesnt exist then do nothing and return
        else:
            return

        # put all lists together to create new animation pose order
        value_list = bottom_list + save_list + top_list

        # reset i
        i = 1
        # clear PoseDict
        self.PoseDict = {}
        # iterate through list to re-fill PoseDict with new order
        for item in value_list:
            self.PoseDict[i] = item
            i = i + 1

        # display updated animation
        self.view_animation()
        # close popup window
        self.toplevel.destroy()
        


def actually_control_inmoov(message):
    # if running on the actual inmoov bot, we can use this callback to bypass ROS and directly hand off the messages
    print(message)
    my_inmoov.inmoov_ros_command(message)



def launch_gui(on_change_callback):
    root = tk.Tk()
    root.title('InMoov Animation Creation GUI')
    app = Application(root, on_change_callback)
    app.mainloop()


if __name__ == '__main__':
    launch_gui(actually_control_inmoov)

