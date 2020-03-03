try:
	import Tkinter as tk
except ModuleNotFoundError:
	import tkinter as tk
import sys
sys.path.append("../scripts/")
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


slider_fullsize = 500
button_padx = 50
button_pady = 8

save_file = '../json/gestures.json'

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
		
		# current_values is initiallized with all zeros
		self.current_values = []
		for p in self.names_all:
			self.current_values.append([])
			for n in p:
				s = my_inmoov.find_servo_by_name(n)
				self.current_values[-1].append(0)
		
		self.butt_init = tk.Button(master, text="INIT", command=self.inmoov_init)
		self.butt_init.grid(row=0, column=0, padx=button_padx, pady=button_pady)
		
		self.butt_left = tk.Button(master, text="Left", command=lambda: self.changemode(0))
		self.butt_left.grid(row=0, column=1, padx=button_padx, pady=button_pady)
		
		self.butt_center = tk.Button(master, text="Center", command=lambda: self.changemode(1))
		self.butt_center.grid(row=0, column=2, padx=button_padx, pady=button_pady)
		
		self.butt_right = tk.Button(master, text="Right", command=lambda: self.changemode(2))
		self.butt_right.grid(row=0, column=3, padx=button_padx, pady=button_pady)
		
		self.butt_off = tk.Button(master, text="OFF", command=self.inmoov_off)
		self.butt_off.grid(row=0, column=4, padx=button_padx, pady=button_pady)

		self.name_entry = tk.Entry(master)
		self.name_entry.grid(row=0, column=5, pady=button_pady)

		self.butt_save = tk.Button(master, text="SAVE", command=self.save_values)
		self.butt_save.grid(row=0, column=6, padx=button_padx, pady=button_pady, sticky=tk.W)
		
		self.defaultcolor = self.butt_off.cget("background")
		print(self.defaultcolor)
		
		# Scale.config(state=tk.DISABLED)
		# Scale.config(state=tk.NORMAL)
		# Label.config(text="asdf")
		self.sliders = []
		self.labels = []
		for i in range(10):
			w = tk.Label(master, text="Hello Tkinter!" + str(i))
			w.grid(row=i+1, column=0)
			self.labels.append(w)
			s = tk.Scale(master, from_=0, to=200, length=600, tickinterval=30, orient=tk.HORIZONTAL, command=self.get_changed_sliders)
			s.grid(row=i+1, column=1, columnspan=4)
			
			self.sliders.append(s)
			# w.get() to return current slider val
			# w.set(x) to set initial value
			# resolution: default 1, set lower for floatingpoint
			# command: callback, gets value as only arg
		
		self.changemode(0)
		pass
	
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
		for n in range(len(self.names_all[self.mode])):
			# overwrite labels
			name = self.names_all[self.mode][n]
			self.labels[n].config(text=name)
			s = my_inmoov.find_servo_by_name(name)
			# min_angle, max_angle
			tick = (s.max_angle - s.min_angle) / 4
			# overwrite ranges & actual position of sliders
			self.sliders[n].config(state=tk.NORMAL, from_=s.min_angle, to=s.max_angle, tickinterval=tick, length=slider_fullsize)
			self.sliders[n].set(self.current_values[self.mode][n])
		for b in range(n+1, 10):
			# disable all other sliders
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
			self.sliders[n].config(state=tk.NORMAL, length=slider_fullsize)
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

