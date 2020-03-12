#!/usr/bin/env python
try:
    import Tkinter as tk
    import tkFont as tkFont
except ModuleNotFoundError:
    import tkinter as tk
    from tkinter import font as tkFont
import json_parsing as jp
from os.path import join, dirname
whereami = dirname(__file__)

import time
import animation_executor as ae

filename_pose = join(whereami, '../json/pose.json')
filename_animation = join(whereami, '../json/animations.json')

button_padx = 5
button_pady = 5

## TODO: add some self checking measures, mostly for pop ups at this point
## TODO: finish comments
## TODO: possibly re-label some items to make more sense

class Application(tk.Frame):
    def __init__(self, master = None):
        tk.Frame.__init__(self, master)

        self.animation_frame = tk.Frame(master,borderwidth=2, relief="solid")
        self.animation_frame.grid(row=1, column =0,padx = 5,pady = 5, sticky = 'wens')
        self.pose_frame = tk.Frame(master, borderwidth=2, relief="solid")
        self.pose_frame.grid(row = 1, column = 1,padx = 5,pady = 5,sticky = 'swne')
        self.button_frame = tk.Frame(self.animation_frame)
        self.button_frame.grid(row = 6, column =0,padx = 5, pady = 10, sticky = 'we', columnspan = 3)
        self.pose_button_frame = tk.Frame(self.animation_frame, borderwidth=1, relief="solid")
        self.pose_button_frame.grid(row = 7, column =0,padx = 5, pady = 10, sticky = 'we', columnspan = 3, ipadx  =5, ipady = 5)

        self.on_off_load = False

        # dictionary to hold all the poses in the animation
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

        self.animation.set(self.AniList[0])

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

        self.button_frame.grid_columnconfigure(0, weight=1)
        self.button_frame.grid_columnconfigure(1, weight=1)
        self.button_frame.grid_columnconfigure(2, weight=1)
        self.pose_button_frame.grid_columnconfigure(0, weight=1)
        self.pose_button_frame.grid_columnconfigure(1, weight=1)
        self.pose_button_frame.grid_columnconfigure(2, weight=1)

        tk.Label(self.pose_frame, text = 'Scroll to view selected pose values:').grid(row = 0, column = 0, sticky = 'w',padx = button_padx)
        
        self.pose_listbox = tk.Listbox(self.pose_frame, width=40, height=26)
        self.pose_listbox.grid(row = 1,column = 0, sticky = 'nswe', padx = 5)

        self.view_pose(self.variable.get())
        
    def add_pose(self):
        try:
            hold_time = float(self.entry_time.get())
            if hold_time > 0:
                self.PoseDict[(self.current_pose)] = [self.variable.get(), hold_time]
                self.view_animation()
                self.entry_time.delete(0, 'end')
                self.current_pose = self.current_pose + 1
            else:
                self.display_error('add_error')
                self.entry_time.delete(0, 'end')
        except:
            self.display_error('add_error')
            self.entry_time.delete(0, 'end')

    def save_animation(self):
        if len(self.entry_save.get()) != 0:
            jp.add_object_to_json(filename_animation, self.entry_save.get(), self.PoseDict)
            print(self.entry_save.get() + 'animation has been saved.')
            self.__init__()
        else:
            self.display_error('save_error')

    def display_error(self, error_type):
        toplevel = tk.Toplevel()
        toplevel.title("ERROR")
        if error_type == 'add_error':
            display_string = 'Please provide a numerical value for pose hold time.'
        else:
            display_string = 'Please provide an animation name.'

        tk.Label(toplevel, text = "ERROR!", fg = 'red').grid(row=0, column=0, sticky = 'we')
        tk.Label(toplevel, text = display_string).grid(row = 1, column = 0)
        tk.Label(toplevel, text = "Click 'x' in top right corner to exit this window.").grid(row = 2, column = 0)

    def view_pose(self, pose_name):
        self.pose_listbox.delete(0,tk.END)
        listFont = tkFont.Font(font=self.pose_listbox.cget("font"))
        spaceLength = listFont.measure(" ")
        spacing = 30 * spaceLength
        pose = jp.read_json(filename_pose)
        servo_list = []
        pwm_list = []
        
        for servo, pwm_value in sorted(pose[self.variable.get()].items()):
            servo_list.append(servo)
            pwm_list.append(pwm_value)

        leftLengths = [listFont.measure(s) for s in servo_list]
        longestLength = max(leftLengths)

        for i in range(len(servo_list)):
            if i%2 == 0:
                color = 'gray95'
            else:
                color = 'white'
            neededSpacing = longestLength + spacing - leftLengths[i]
            spacesToAdd = int(round(neededSpacing/spaceLength))
            self.pose_listbox.insert(tk.END, "  " + servo_list[i] + spacesToAdd*" " + str(pwm_list[i]))
            self.pose_listbox.itemconfig(i, {'bg':color})
            
    def view_animation(self):
        pose_list = []
        self.listbox_widget.delete(0,tk.END)
        i = 1
        for pose, pose_info in sorted(self.PoseDict.items()):
            pose_string = "{}. Hold {} pose for {} second(s)".format(i, pose_info[0], pose_info[1])
            pose_list.append(pose_string)
            i = i + 1
        for entry in pose_list:
            self.listbox_widget.insert(tk.END, entry)

    def clear_animation(self):
        self.PoseDict = {}
        self.listbox_widget.delete(0,tk.END)
        self.current_pose = 1
        self.view_animation()

    def execute_poses(self):
        for pose_id, pose_info in sorted(self.PoseDict.items()):
            print("\n********* Executing pose {} *********\n".format(str(pose_info[0])))
            ae.do_pose(pose_info[0], pose_info[1])
        print("\nANIMATION COMPLETE!\n")

    def show_load_animation(self):
        self.on_off_load = not self.on_off_load
        if self.on_off_load:
            self.ani_select.grid(row=1, column=0, columnspan = 2,padx = 5, sticky = 'we', pady = (20,0))
            self.animation_confirm.grid(row = 1, column = 2,padx = 5, sticky = 'we', pady = (20, 0))
        else:
            self.ani_select.grid_remove()
            self.animation_confirm.grid_remove()

    def edit_time_view(self):
        try:
            current_pose_selection = int(self.listbox_widget.curselection()[0]) + 1
        except:
            return
        info = self.PoseDict.get(current_pose_selection)
        self.listbox_widget.selection_clear(0, tk.END)
        toplevel = tk.Toplevel()
        toplevel.title("Change Pose Duration")
        toplevel.grid_columnconfigure(0, weight=1)
        toplevel.grid_columnconfigure(1, weight=1)
        toplevel.grid_columnconfigure(2, weight=1)
        string1 = "Current Time Duration for {} pose is {} second(s).".format(info[0], info[1])
        tk.Label(toplevel, text = string1).grid(row=0, column=0, columnspan = 3)
        tk.Label(toplevel, text = "Change to:").grid(row = 1 , column = 0)
        time_entry = tk.Entry(toplevel)
        time_entry.grid(row = 1, column = 1)
        tk.Button(toplevel, text = "OK", command = lambda: self.change_time(current_pose_selection, time_entry.get())).grid(row = 1, column = 2)
        tk.Label(toplevel, text = "Remember to close this window before changing any other time value").grid(row=2, column=0, columnspan = 3)

    def load_animation(self):
        self.PoseDict = {}
        i = 0
        for pose_num, pose_info in sorted(self.ani_data[self.animation.get()].items()):
            self.PoseDict[i] = pose_info
            i = i + 1
        self.view_animation()
        self.entry_save.delete(0, 'end')
        self.entry_save.insert(0,self.animation.get())
        self.on_off_load  = True
        self.show_load_animation()

    def set_animation(self, value):
        index_value = self.AniList.index(value)
        self.animation.set(self.AniList[index_value])

    def change_time(self, pose_index, new_time):
        self.PoseDict[pose_index][1] = float(new_time)
        self.view_animation()

    def delete_pose(self):
        try:
            current_pose_selection = int(self.listbox_widget.curselection()[0]) + 1
        except:
            return
        key_list = []
        value_list = []
        info = self.PoseDict.get(current_pose_selection)
        self.listbox_widget.selection_clear(0, tk.END)

        for key, value in sorted(self.PoseDict.items()):
            value_list.append(value)

        del value_list[current_pose_selection-1]
        self.PoseDict = {}

        i = 1
        for x in value_list:
            self.PoseDict[i] = x
            i = i+1
            
        self.view_animation()

    def edit_order(self):
        try:
            current_pose_selection = int(self.listbox_widget.curselection()[0]) + 1
        except:
            return
        info = self.PoseDict.get(current_pose_selection)
        self.listbox_widget.selection_clear(0, tk.END)
        toplevel = tk.Toplevel()
        toplevel.title("Change Pose Position")
        toplevel.grid_columnconfigure(0, weight=1)
        toplevel.grid_columnconfigure(1, weight=1)
        toplevel.grid_columnconfigure(2, weight=1)
        string1 = "{}is currently in position {} in the animation.".format(info[0], current_pose_selection)
        tk.Label(toplevel, text = string1).grid(row=0, column=0, columnspan = 3)
        tk.Label(toplevel, text = "Change to:").grid(row = 1 , column = 0)
        number_entry = tk.Entry(toplevel)
        number_entry.grid(row = 1, column = 1)
        tk.Button(toplevel, text = "OK", command = lambda: self.reorder_poses(int(number_entry.get()), current_pose_selection)).grid(row = 1, column = 2)
        tk.Label(toplevel, text = "Remember to close this window before changing any other position value").grid(row=2, column=0, columnspan = 3)

    ## FINISH THIS
    def reorder_poses(self, new_position, old_position):
        new_position = new_position -1
        old_position = old_position - 1
        value_list = []
        bottom_list = []
        top_list = []
        save_list = []

        for key, value in sorted(self.PoseDict.items()):
            value_list.append(value)
        
        save_list.append(value_list[old_position])

        if new_position < old_position:
            bottom_list = value_list[:(new_position)]
            top_list = value_list[new_position:]
            top_list.remove(save_list[0])
        elif new_position > old_position:
            bottom_list = value_list[:(new_position+1)]
            top_list = value_list[new_position+1:]
            bottom_list.remove(save_list[0])
        else:
            return

        value_list = bottom_list + save_list + top_list

        i = 1
        self.PoseDict = {}
        for item in value_list:
            self.PoseDict[i] = item
            i = i + 1

        self.view_animation()
        


if __name__ == '__main__':
        root = tk.Tk()
        root.title('InMoov Animation Creation GUI')
        app = Application(master=root)
        app.mainloop()
