import tkinter as tk

import Inmoov

inmoov = Inmoov.Inmoov()

# slider
# button
# text label
# grid structure



class Application(tk.Frame):
	def __init__(self, master=None):
		tk.Frame.__init__(self, master)
		# self.pack()
		
		self.hi_there = tk.Button(master)
		self.hi_there["text"] = "Hello World\n(click me)"
		self.hi_there["command"] = self.say_hi
		# self.hi_there.pack(side="top")
		self.hi_there.grid(row=0, column=0)
		
		self.QUIT = tk.Button(master, text="QUIT", fg="red", command=root.destroy)
		# self.QUIT.pack(side="bottom")
		self.QUIT.grid(row=0, column=1)
		
		self.sliders = []
		self.labels = []
		for i in range(1,10):
			w = tk.Label(master, text="Hello Tkinter!" + str(i))
			w.grid(row=i, column=0)
			self.labels.append(w)
			s = tk.Scale(master, from_=0, to=200, length=600, tickinterval=30, orient=tk.HORIZONTAL)
			s.grid(row=i, column=1)
			self.sliders.append(s)
			# w.get() to return current slider val
			# w.set(x) to set initial value
			# resolution: default 1, set lower for floatingpoint
			# command: callback, gets value as only arg
		
		
		
		
		pass
	
	def say_hi(self):
		print("hi there, everyone!")

root = tk.Tk()
app = Application(master=root)
app.mainloop()