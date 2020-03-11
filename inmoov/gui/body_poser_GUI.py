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


# sliders and checkboxes are only enabled/disabled during init and changetab
# if a servo is disabled, its slider/box is disabled. period. no getting around it.

# slider values are tracked with current_values, used to update sliders when changing tabs
# also used to know which slider has changed when one changes
# checkbox states are tracked with current_checkbox_states in the same way
# if a box is unchecked, that servo will not be part of any saved pose
# if a servo name is red, that servo is OFF and depowered. moving that servo will re-power it.

label_width = 20
slider_width = 500
button_padx = 10
button_pady = 10
button_width = 5        # not sure what units this is using
textentry_width = 20

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
        # current_checkbox_states is same as current_values but for the checkboxes_vars
        # disabled servos begin with this checkbox off
        self.pose_init = {}
        self.current_checkbox_states = []
        self.current_onoff_states = []
        for p in self.names_all:
            boxes = []
            onoff = []
            for n in p:
                s = my_inmoov.find_servo_by_name(n)
                self.pose_init[n] = s.default_angle
                boxes.append(int(not s.disabled))
                onoff.append(int(not s.disabled))
            self.current_checkbox_states.append(boxes)
            self.current_onoff_states.append(onoff)

        print(self.current_checkbox_states)
        
        # applying an "empty pose" doesn't change any sliders, but it unchecks all checkboxes
        self.pose_off = {}
        
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
        
        # self.frame_tab_buttons = tk.Frame(self.frame_biggroup1, highlightbackground="black", highlightthickness=1)
        self.frame_tab_buttons = tk.Frame(self.frame_biggroup1, relief=tk.RAISED, borderwidth=1)
        self.frame_tab_buttons.pack(side=tk.LEFT, padx=10, pady=10)
        self.butt_left = tk.Button(self.frame_tab_buttons, text="Left", width=button_width, command=lambda: self.change_tab(0))
        self.butt_left.pack(side=tk.LEFT, padx=button_padx, pady=button_pady)
        self.butt_center = tk.Button(self.frame_tab_buttons, text="Center", width=button_width, command=lambda: self.change_tab(1))
        self.butt_center.pack(side=tk.LEFT, padx=button_padx, pady=button_pady)
        self.butt_right = tk.Button(self.frame_tab_buttons, text="Right", width=button_width, command=lambda: self.change_tab(2))
        self.butt_right.pack(side=tk.LEFT, padx=button_padx, pady=button_pady)
        
        self.frame_command_buttons = tk.Frame(self.frame_biggroup1, relief=tk.RAISED, borderwidth=1)
        self.frame_command_buttons.pack(side=tk.LEFT, pady=10)
        self.butt_init = tk.Button(self.frame_command_buttons, text="INIT", width=button_width, command=self.allservos_init)
        self.butt_init.pack(side=tk.LEFT, padx=button_padx, pady=button_pady)
        self.butt_off = tk.Button(self.frame_command_buttons, text="OFF", width=button_width, command=self.allservos_off)
        self.butt_off.pack(side=tk.LEFT, padx=button_padx, pady=button_pady)

        self.frame_posesave_widgets = tk.Frame(self.frame_biggroup1, relief=tk.RAISED, borderwidth=1)
        self.frame_posesave_widgets.pack(side=tk.LEFT, padx=10, pady=10)
        self.name_entry = tk.Entry(self.frame_posesave_widgets, width=textentry_width)
        self.name_entry.pack(side=tk.LEFT, padx=button_padx, pady=button_pady)
        self.butt_save = tk.Button(self.frame_posesave_widgets, text="Save", width=button_width, command=self.save_values)
        self.butt_save.pack(side=tk.LEFT, pady=button_pady)
        self.butt_load = tk.Button(self.frame_posesave_widgets, text="Load", width=button_width, command=self.load_values)
        self.butt_load.pack(side=tk.LEFT, padx=button_padx, pady=button_pady)

        # the default color always looks the same but has different names on different platforms
        self.defaultcolor_buttonback = self.butt_off.cget("background")
        print(self.defaultcolor_buttonback)


        # Scale.config(state=tk.DISABLED)
        # Scale.config(state=tk.NORMAL)
        # Label.config(text="asdf")

        ###########################################################################
        # build & fill the scrollbar region
        
        # frame that contains everything that isn't the above buttons
        self.frame_biggroup2 = tk.Frame(master)
        self.frame_biggroup2.pack(fill=tk.BOTH, expand=True)
        
        # canvas holds a frame widget
        canvas_width = int(slider_width + (7.5 * label_width) + 23) # set the default and minimum width
        self.canvas = tk.Canvas(self.frame_biggroup2, height=canvas_height, width=canvas_width)
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # scrollbar
        # change the canvas when the scrollbar is scrolled
        self.scrollbar = tk.Scrollbar(self.frame_biggroup2, command=self.canvas.yview)
        self.scrollbar.pack(side=tk.LEFT, fill='y')
        # set the scrollbar when something changes the canvas (window resizing)
        self.canvas.configure(yscrollcommand=self.scrollbar.set)
        # not totally sure what this does, gets called when the canvas is resized (window resizing)
        self.canvas.bind('<Configure>', self.update_canvas_size)

        # --- put frame in canvas ---
        self.frame_servolist = tk.Frame(self.canvas)
        self.canvas.create_window((0, 0), window=self.frame_servolist, anchor='nw')

        # --- add widgets in frame ---
        self.sliders = []
        self.labels = []
        self.checkboxes = []
        self.checkboxes_vars = []
        for i in range(self.maxtablistlength):
            w = tk.Label(self.frame_servolist, text="Hello Tkinter!" + str(i), width=label_width)
            w.grid(row=i, column=0)
            self.labels.append(w)
            
            s = tk.Scale(self.frame_servolist, from_=0, to=200, length=slider_width, tickinterval=30,
                         orient=tk.HORIZONTAL, command=self.get_changed_sliders)
            s.grid(row=i, column=1)
            self.sliders.append(s)
            
            var = tk.IntVar()
            c = tk.Checkbutton(self.frame_servolist, variable=var, text="", command=self.get_changed_checkbox)
            # var.set(1) # turn the button on by default
            c.grid(row=i, column=2)
            self.checkboxes.append(c)
            self.checkboxes_vars.append(var)

            # w.get() to return current slider val
            # w.set(x) to set initial value
            # resolution: default 1, set lower for floatingpoint
            # command: callback, gets value as only arg

        # the default color always looks the same but has different names on different platforms
        self.defaultcolor_font = self.labels[0].cget("fg")
        print(self.defaultcolor_font)

        self.change_tab(0)
        # DONE WITH GUI INIT
        pass

    def update_canvas_size(self, event):
        # not sure what this does but it is called whenever the window is resized, and its necessary
        # print("configure canvas")
        # update scrollregion after starting 'mainloop' when all widgets are in canvas
        self.canvas.configure(scrollregion=self.canvas.bbox('all'))

    def change_tab(self, newmode):
        if newmode==0:
            self.butt_left.config(bg="red")
        else:
            self.butt_left.config(bg=self.defaultcolor_buttonback)
        if newmode==1:
            self.butt_center.config(bg="red")
        else:
            self.butt_center.config(bg=self.defaultcolor_buttonback)
        if newmode==2:
            self.butt_right.config(bg="red")
        else:
            self.butt_right.config(bg=self.defaultcolor_buttonback)

        self.mode = newmode
        n = 0
        for n in range(len(self.names_all[self.mode])):
            # overwrite labels
            name = self.names_all[self.mode][n]
            s = my_inmoov.find_servo_by_name(name)
            self.labels[n].config(text=name)
            # sete text color depending on on/off state
            if self.current_onoff_states[self.mode][n]:
                self.labels[n].config(fg=self.defaultcolor_font)
            else:
                self.labels[n].config(fg="red")
            # update checkboxes
            state = self.current_checkbox_states[self.mode][n]
            if s.disabled:
                self.checkboxes_vars[n].set(0)
                self.checkboxes[n].config(state=tk.DISABLED)
            else:
                self.checkboxes_vars[n].set(state)
                self.checkboxes[n].config(state=tk.NORMAL)
            # update sliders
            if s.disabled:
                self.sliders[n].config(state=tk.DISABLED, length=0)
            else:
                tick = float(s.max_angle - s.min_angle) / 4.0
                # overwrite ranges & actual position of sliders
                self.sliders[n].config(from_=s.min_angle, to=s.max_angle, tickinterval=tick,
                                       state=tk.NORMAL, length=slider_width)
                self.sliders[n].set(self.current_values[self.mode][n])
        for b in range(n+1, self.maxtablistlength):
            # disable all other sliders & wipe their labels
            self.labels[b].config(text="-----")
            self.sliders[b].config(state=tk.DISABLED, length=0)
            self.checkboxes_vars[b].set(0)
            self.checkboxes[b].config(state=tk.DISABLED)

    def allservos_off(self):
        # note that off does not correspond to angle=0, so there's not much point updating the displayed sliders
        # make the actual inmoov turn off for ALL servos, not just currently active tab
        self.on_change_callback("off!")
        # pose_off doesn't change any slider values, but it does uncheck all checkboxes
        self.apply_pose_to_gui(self.pose_off)
        # turn all onoff states to off, and update all visible sliders
        for l in self.labels:
            l.config(fg="red")
        self.current_onoff_states = []
        for p in self.names_all:
            self.current_onoff_states.append([0] * len(p))


    def allservos_init(self):
        # make the actual inmoov run init for ALL servos, not just currently active tab
        self.on_change_callback("init!")
        # set all current_values to defaults
        # set all displayed sliders to the corresponding defaults
        self.apply_pose_to_gui(self.pose_init)
    
    def get_changed_checkbox(self):
        for i, n in enumerate(self.names_all[self.mode]):
            c = self.checkboxes_vars[i].get()
            if c != self.current_checkbox_states[self.mode][i]:
                # if it has changed, then update its tracked val
                self.current_checkbox_states[self.mode][i] = c
        print(self.current_checkbox_states)

    def get_changed_sliders(self, x):
        for i, n in enumerate(self.names_all[self.mode]):
            c = self.sliders[i].get()
            if c != self.current_values[self.mode][i]:
                # if it has changed, then update its tracked val & send it via ROS
                self.current_values[self.mode][i] = c
                self.on_change_callback("servo!" + n + "!" + str(c))
                # also update the text label color
                self.current_onoff_states[self.mode][i] = 1
                self.labels[i].config(fg=self.defaultcolor_font)
                # special corner case: moving one torso slider should also move the other!
                if n == "left_torso":
                    v = self.names_all[self.mode].index("right_torso")
                    self.current_values[self.mode][v] = c
                    self.sliders[v].set(c)
                    self.on_change_callback("servo!right_torso" + "!" + str(c))
                elif n == "right_torso":
                    v = self.names_all[self.mode].index("left_torso")
                    self.current_values[self.mode][v] = c
                    self.sliders[v].set(c)
                    self.on_change_callback("servo!left_torso" + "!" + str(c))


    def save_values(self):
        # uses the name entered in the text box to append a new object onto the end of the existing "gestures.json" file
        gesture = {}
        save_name = self.name_entry.get()
        if len(save_name) == 0:
            # TODO: popup!
            print('Please enter a pose name.')
            return
        current_gestures = jp.read_json(save_file)
        if save_name in current_gestures:
            print('Pose already exsists')
            # TODO: popup! can we get a yes/no repsonse from the user?
            return
        for n in range(len(self.names_all)):
            for m in range(len(self.names_all[n])):
                if self.current_checkbox_states[n][m]: # only add it to the pose if its checkbox is checked
                    c = self.current_values[n][m] # get current slider value
                    gesture[self.names_all[n][m]] = c # save it into dict
        jp.add_object_to_json(save_file, save_name, gesture)
        self.name_entry.delete(0, 'end') # delete the text in the text-entry box
        totalnumservos = sum([len(x) for x in self.names_all])
        print("Pose saved as '%s', sets %d / %d servos" % (save_name, len(gesture), totalnumservos))
            
    def load_values(self):
        # read the json, get the specified pose
        load_name = self.name_entry.get()
        if len(load_name) == 0:
            # todo popup!
            print('Please enter a pose name.')
            return
        current_poses = jp.read_json(save_file)
        if load_name not in current_poses:
            # todo popup!
            print('Specified pose does not exist.')
            return
        loading_pose = current_poses[load_name]
        # load/apply the pose to the GUI:
        self.apply_pose_to_gui(loading_pose)
        # send the pose to the physical robot
        for name,val in loading_pose.items():
            self.on_change_callback("servo!" + str(name) + "!" + str(float(val)))
        return

    def apply_pose_to_gui(self, dictpose):
        # given a dict-pose, apply it to the gui
        # firstly, ignore all disabled servos. don't touch their sliders or checkboxes.
        # second, if a servo IS in the pose, then update its slider and set its checkbox=set.
        # third, if a servo IS NOT in the pose, then set it checkbox=unset.
        for n in range(len(self.names_all)):
            for m in range(len(self.names_all[n])):
                s = my_inmoov.find_servo_by_name(self.names_all[n][m])
                if s.disabled:
                    # dont change disabled servos at all, whether in background or active tab
                    continue
                if self.names_all[n][m] in dictpose:
                    # if this servo is in the pose, then set its checkbox to checked
                    self.current_checkbox_states[n][m] = 1
                    # if this servo is in the pose, then set its text color to black
                    self.current_onoff_states[n][m] = 1
                    # if this servo is in the pose, then update its slider to match the value in it
                    self.current_values[n][m] = dictpose[self.names_all[n][m]]
                    if n == self.mode:
                        # if in the visible tab, update the visible slider and visible checkbox and visible label
                        self.labels[m].config(fg=self.defaultcolor_font)
                        self.sliders[m].set(s.default_angle)
                        self.checkboxes_vars[m].set(1)
                else:
                    # if the servo is not in the pose, then set its checkbox to unchecked
                    self.current_checkbox_states[n][m] = 0
                    if n == self.mode:
                        # if in the visible tab, update the visible checkbox and label
                        self.checkboxes_vars[m].set(0)
                        self.labels[m].config(fg="red")
                    # if the servo is not in the pose, then don't touch its sliders at all


def actually_control_inmoov(message):
    # if running on the actual inmoov bot, we can use this callback to bypass ROS and directly hand off the messages
    print(message)
    my_inmoov.set_servo_ros(message)

def launch_gui(on_change_callback):
    root = tk.Tk()
    app = Application(root, on_change_callback)
    app.mainloop()


if __name__ == '__main__':
    launch_gui(actually_control_inmoov)
