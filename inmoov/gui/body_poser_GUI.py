#!/usr/bin/env python
try:
    import Tkinter as tk
except ModuleNotFoundError:
    import tkinter as tk
import sys
from os.path import join, dirname
whereami = dirname(__file__)
scriptsdir = join(whereami, "../scripts/")
sys.path.append(scriptsdir)
import Inmoov
import json_parsing as jp


# slider
# button
# text label
# grid structure

# also 3 corresponding lists of current values, initialized with defaults
# 3 lists of names of servos: right, left, center
# for each list, search for those names & add to list
# make 2 columns: left is names (of one list), right is sliders (columnspan=2)
# 3 buttons at the top that let you change the displayed list, + 1 button for "reset" or "init" and 1 for "off"
# all sliders have the same callback function: check all sliders to see which slider(s) have changed and send them over ROS
# know it is changed by checking against the appropriate "current value" list

label_width = 20
slider_width = 300
button_padx = 10
button_pady = 10

button_width = 5        # not sure what units this is using

canvas_height = 400

save_file = join(whereami, '../json/pose.json')

# create the INMOOV as a lazy way to parse the JSON and stuff
# or when running locally, this actually instantiates & controls the servos
my_inmoov = Inmoov.Inmoov()


class Application(tk.Frame):
    def __init__(self, master, on_change_callback):
        tk.Frame.__init__(self, master)

        # this is the function to run when something changes and its time to send a message to ROS or inmoov
        # this is a function that takes a single string
        self.on_change_callback = on_change_callback

        self.mode = 0	# mode 0=left, 1=center, 2=right

        self.names_all = [["left_wrist",
                           "left_pinky",
                           "left_ring",
                           "left_mid",
                           "left_index",
                           "left_thumb",
                           "left_elbow",
                           "left_shoulder_lift_out",
                           "left_shoulder_lift_front",
                           "left_arm_rotate"],
                          ["head_x",
                           "head_y",
                           "jaw",
                           "left_torso",
                           "right_torso"],
                          ["right_wrist",
                            "right_pinky",
                            "right_ring",
                            "right_mid",
                            "right_index",
                            "right_thumb",
                            "right_elbow",
                            "right_shoulder_lift_out",
                            "right_shoulder_lift_front",
                            "right_arm_rotate"]]

        self.maxtablistlength = 0     # the most servos in a tab
        for namelist in self.names_all:
            self.maxtablistlength = max(self.maxtablistlength, len(namelist))
        # current_values is initiallized with all zeros, in the same shape as self.names_all
        # used to determine which changed when something changed, also to know what to display when loading a tab
        self.current_values = []
        for p in self.names_all:
            self.current_values.append([0] * len(p))

        #####################################
        # begin GUI setup
        # structure:
        # one frame for all buttons stacked vertically with one frame for the scrollbar stuff
        # inside buttons frame, everything packed horizontally
        # buttons grouped with visible borders to show things that are related:
        # left/right/center buttons inside a frame with a visible border
        # init/off buttons inside a frame with a visible border
        # name/save field/button inside a frame with a visible border
        
        # buttons for control & stuff
        self.frame_biggroup1 = tk.Frame(master)
        self.frame_biggroup1.pack(side=tk.TOP)
        
        self.frame_tab_buttons = tk.Frame(self.frame_biggroup1, highlightbackground="black", highlightthickness=1)
        # self.frame_tab_buttons = tk.Frame(self.frame_biggroup1, relief=tk.RAISED, borderwidth=1)
        self.frame_tab_buttons.pack(side=tk.LEFT, padx=10, pady=10)
        self.butt_left = tk.Button(self.frame_tab_buttons, text="Left", width=button_width, command=lambda: self.changemode(0))
        self.butt_left.pack(side=tk.LEFT, padx=button_padx, pady=button_pady)
        self.butt_center = tk.Button(self.frame_tab_buttons, text="Center", width=button_width, command=lambda: self.changemode(1))
        self.butt_center.pack(side=tk.LEFT, padx=button_padx, pady=button_pady)
        self.butt_right = tk.Button(self.frame_tab_buttons, text="Right", width=button_width, command=lambda: self.changemode(2))
        self.butt_right.pack(side=tk.LEFT, padx=button_padx, pady=button_pady)
        
        self.frame_command_buttons = tk.Frame(self.frame_biggroup1, relief=tk.RAISED, borderwidth=1)
        self.frame_command_buttons.pack(side=tk.LEFT, pady=10)
        self.butt_init = tk.Button(self.frame_command_buttons, text="INIT", width=button_width, command=self.inmoov_init)
        self.butt_init.pack(side=tk.LEFT, padx=button_padx, pady=button_pady)
        self.butt_off = tk.Button(self.frame_command_buttons, text="OFF", width=button_width, command=self.inmoov_off)
        self.butt_off.pack(side=tk.LEFT, padx=button_padx, pady=button_pady)

        self.frame_posesave_widgets = tk.Frame(self.frame_biggroup1, relief=tk.RAISED, borderwidth=1)
        self.frame_posesave_widgets.pack(side=tk.LEFT, padx=10, pady=10)
        self.name_entry = tk.Entry(self.frame_posesave_widgets)
        self.name_entry.pack(side=tk.LEFT, padx=button_padx, pady=button_pady)
        self.butt_save = tk.Button(self.frame_posesave_widgets, text="SAVE", width=button_width, command=self.save_values)
        self.butt_save.pack(side=tk.LEFT, padx=button_padx, pady=button_pady)

        # the default color always looks the same but has different names on different platforms
        self.defaultcolor = self.butt_off.cget("background")
        print(self.defaultcolor)

        # Scale.config(state=tk.DISABLED)
        # Scale.config(state=tk.NORMAL)
        # Label.config(text="asdf")

        ###########################################################################
        # build & fill the scrollbar region
        
        # frame that contains everything that isn't the above buttons
        self.frame_biggroup2 = tk.Frame(master)
        self.frame_biggroup2.pack(fill=tk.BOTH, expand=True)
        # self.frame_biggroup2.pack(side=tk.TOP)
        
        # canvas holds a frame widget
        canvas_width = int(slider_width + (7.5 * label_width)) # set the default and minimum width
        self.canvas = tk.Canvas(self.frame_biggroup2, height=canvas_height, width=canvas_width)
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        # self.canvas.pack(side=tk.LEFT)

        # scrollbar
        # change the canvas when the scrollbar is scrolled
        self.scrollbar = tk.Scrollbar(self.frame_biggroup2, command=self.canvas.yview)
        self.scrollbar.pack(side=tk.LEFT, fill='y')
        # set the scrollbar when something changes the canvas (window resizing)
        self.canvas.configure(yscrollcommand=self.scrollbar.set)
        # not totally sure what this does, gets called when the canvas is resized (window resizing)
        self.canvas.bind('<Configure>', self.on_configure)

        # --- put frame in canvas ---
        self.frame_servolist = tk.Frame(self.canvas)
        self.canvas.create_window((0, 0), window=self.frame_servolist, anchor='nw')

        # --- add widgets in frame ---
        self.sliders = []
        self.labels = []
        for i in range(self.maxtablistlength):
            w = tk.Label(self.frame_servolist, text="Hello Tkinter!" + str(i), width=label_width)
            w.grid(row=i, column=0)
            self.labels.append(w)
            s = tk.Scale(self.frame_servolist, from_=0, to=200, length=slider_width, tickinterval=30, orient=tk.HORIZONTAL, command=self.get_changed_sliders)
            s.grid(row=i, column=1)
            self.sliders.append(s)
            # TODO: checkboxes
            
            # w.get() to return current slider val
            # w.set(x) to set initial value
            # resolution: default 1, set lower for floatingpoint
            # command: callback, gets value as only arg

        self.changemode(0)
        # DONE WITH GUI INIT
        pass

    def on_configure(self, event):
        print("configure canvas")
        # update scrollregion after starting 'mainloop' when all widgets are in canvas
        self.canvas.configure(scrollregion=self.canvas.bbox('all'))

    def changemode(self, newmode):
        if newmode==0:
            self.butt_left.config(bg="red")
        else:
            self.butt_left.config(bg=self.defaultcolor)
        if newmode==1:
            self.butt_center.config(bg="red")
        else:
            self.butt_center.config(bg=self.defaultcolor)
        if newmode==2:
            self.butt_right.config(bg="red")
        else:
            self.butt_right.config(bg=self.defaultcolor)

        self.mode = newmode
        n = 0
        # TODO: treat json-disabled servos differently somehow?
        for n in range(len(self.names_all[self.mode])):
            # overwrite labels
            name = self.names_all[self.mode][n]
            self.labels[n].config(text=name)
            s = my_inmoov.find_servo_by_name(name)
            # min_angle, max_angle
            tick = float(s.max_angle - s.min_angle) / 4.0
            # overwrite ranges & actual position of sliders
            self.sliders[n].config(state=tk.NORMAL, from_=s.min_angle, to=s.max_angle, tickinterval=tick, length=slider_width)
            self.sliders[n].set(self.current_values[self.mode][n])
        for b in range(n+1, self.maxtablistlength):
            # disable all other sliders & wipe their labels
            self.labels[b].config(text="-----")
            self.sliders[b].config(state=tk.DISABLED, length=0)


    def inmoov_off(self):
        # make the actual inmoov turn off for ALL servos, not just currently active tab
        self.on_change_callback("off")
        # disable all sliders
        #for v in self.sliders:
        #	v.config(state=tk.DISABLED, length=0)

    def inmoov_init(self):
        # make the actual inmoov run init for ALL servos, not just currently active tab
        self.on_change_callback("init")
        # enable all sliders
        for n in range(len(self.names_all[self.mode])):
            self.sliders[n].config(state=tk.NORMAL, length=slider_width)
        # set all current_values to defaults
        # set all displayed sliders to the corresponding defaults
        for p in range(len(self.names_all)):
            for n in range(len(self.names_all[p])):
                s = my_inmoov.find_servo_by_name(self.names_all[p][n])
                # set all my current value trackers to be defaults
                self.current_values[p][n] = s.default_angle
                if p == self.mode:
                    # set current values of all active sliders to their defaults
                    self.sliders[n].set(s.default_angle)

    def get_changed_sliders(self, x):
        for i, n in enumerate(self.names_all[self.mode]):
            c = self.sliders[i].get()
            if c != self.current_values[self.mode][i]:
                # if it has changed, then update its tracked val & send it via ROS
                self.current_values[self.mode][i] = c
                # n = self.names_all[self.mode][i]
                self.on_change_callback(n + "!" + str(c))
                # special corner case: moving one torso slider should also move the other!
                if n == "left_torso":
                    v = self.names_all[self.mode].index("right_torso")
                    self.current_values[self.mode][v] = c
                    self.sliders[v].set(c)
                    self.on_change_callback("right_torso" + "!" + str(c))
                elif n == "right_torso":
                    v = self.names_all[self.mode].index("left_torso")
                    self.current_values[self.mode][v] = c
                    self.sliders[v].set(c)
                    self.on_change_callback("left_torso" + "!" + str(c))


    def save_values(self):
        # uses the name entered in the text box to append a new object onto the end of the existing "gestures.json" file
        gesture = {}
        save_name = self.name_entry.get()
        if len(save_name) != 0:
            current_gestures = jp.read_json(save_file)
            if save_name in current_gestures:
                print('Gesture already exsists')
            else:
                for n in range(len(self.names_all)):
                    for m in range(len(self.names_all[n])):
                        c = self.current_values[n][m]
                        gesture[self.names_all[n][m]] = c
                jp.add_object_to_json(save_file, save_name, gesture)
                #jp.pretty_print(save_file)
                self.name_entry.delete(0, 'end')
                print("Gesture saved as '" + save_name + "'")
        else:
            print('Please enter a gesture name.')

def actually_control_inmoov(message):
    # if running on the actual inmoov bot, we can use this callback to bypass ROS and directly hand off the messages
    print(message)
    my_inmoov.set_servo_ros(message)

def launch_gui(on_change_callback):
    root = tk.Tk()
    app = Application(root, on_change_callback)
    app.mainloop()


if __name__ == '__main__':
    # launch_gui(send_with_ros)
    launch_gui(actually_control_inmoov)
