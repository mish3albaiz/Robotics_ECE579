import tkinter as tk
import sys
sys.path.append("../body/src/")
import Inmoov


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

fullsize = 600


class Application(tk.Frame):
	def __init__(self, master=None):
		tk.Frame.__init__(self, master)

		# create the INMOOV as a lazy way to parse the JSON and stuff
		
		self.inmoov = Inmoov.Inmoov()
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
				s = self.inmoov.find_servo_by_name(n)
				self.current_values[-1].append(0)
		
		self.butt_init = tk.Button(master, text="INIT", command=self.inmoov_init)
		self.butt_init.grid(row=0, column=0)
		
		self.butt_left = tk.Button(master, text="Left", command=self.changemode_left)
		self.butt_left.grid(row=0, column=1)
		
		self.butt_center = tk.Button(master, text="Center", command=self.changemode_center)
		self.butt_center.grid(row=0, column=2)
		
		self.butt_right = tk.Button(master, text="Right", command=self.changemode_right)
		self.butt_right.grid(row=0, column=3)
		
		self.butt_off = tk.Button(master, text="OFF", command=self.inmoov_off)
		self.butt_off.grid(row=0, column=4)
		
		self.defaultcolor = "SystemButtonFace"
		
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
		
		self.changemode_left()
		
		
		pass
	
	def changemode_left(self):
		self.butt_left.config(bg="red")
		self.butt_center.config(bg="SystemButtonFace")
		self.butt_right.config(bg="SystemButtonFace")
		self.changemode(0)
	def changemode_center(self):
		self.butt_left.config(bg="SystemButtonFace")
		self.butt_center.config(bg="red")
		self.butt_right.config(bg="SystemButtonFace")
		self.changemode(1)
	def changemode_right(self):
		self.butt_left.config(bg="SystemButtonFace")
		self.butt_center.config(bg="SystemButtonFace")
		self.butt_right.config(bg="red")
		self.changemode(2)
	
	def changemode(self, newmode):
		self.mode = newmode
		n = 0
		for n in range(len(self.names_all[self.mode])):
			# overwrite labels
			name = self.names_all[self.mode][n]
			self.labels[n].config(text=name)
			s = self.inmoov.find_servo_by_name(name)
			# min_angle, max_angle
			tick = (s.max_angle - s.min_angle) / 4
			# overwrite ranges & actual position of sliders
			self.sliders[n].config(state=tk.NORMAL, from_=s.min_angle, to=s.max_angle, tickinterval=tick, length=fullsize)
			self.sliders[n].set(self.current_values[self.mode][n])
		for b in range(n+1, 10):
			# disable all other sliders
			self.labels[b].config(text="-----")
			self.sliders[b].config(state=tk.DISABLED, length=0)
		
	
	def inmoov_off(self):
		# make the actual inmoov turn off
		self.send_with_ros("off")
		# # disable all sliders
		for v in self.sliders:
			v.config(state=tk.DISABLED, length=0)
	
	def inmoov_init(self):
		# make the actual inmoov run init
		self.send_with_ros("init")
		# # enable all sliders
		for n in range(len(self.names_all[self.mode])):
			self.sliders[n].config(state=tk.NORMAL, length=fullsize)
		for p in range(len(self.names_all)):
			for n in range(len(self.names_all[p])):
				s = self.inmoov.find_servo_by_name(self.names_all[p][n])
				# set all my current value trackers to be defaults
				self.current_values[p][n] = s.default_angle
				if p == self.mode:
					# set current values of all active sliders to their defaults
					self.sliders[n].set(s.default_angle)
	
	def get_changed_sliders(self, x):
		for i in range(len(self.names_all[self.mode])):
			c = self.sliders[i].get()
			if c != self.current_values[self.mode][i]:
				# if it has changed, then update its tracked val & send it via ROS
				self.current_values[self.mode][i] = c
				n = self.names_all[self.mode][i]
				self.send_with_ros(n + "!" + str(c))
	
	def send_with_ros(self, message):
		print(message)
		# TODO: ROS everything
	
	
	def say_hi(self):
		print("hi there, everyone!")

root = tk.Tk()
app = Application(master=root)
app.mainloop()

