import tkinter as tk
import json_parsing as jp

filename = '../json/gestures.json'

button_padx = 5
button_pady = 5

class Application(tk.Frame):
    def __init__(self, master = None):
        tk.Frame.__init__(self, master)

        self.OptionList = []
        self.ServoOptionList = []

        self.data = jp.read_json(filename)

        self.delete = False

        for gesture in self.data:
            self.OptionList.append(gesture)

        for servo in self.data[self.OptionList[0]].keys():
            self.ServoOptionList.append(servo)

        self.variable = tk.StringVar(master)
        self.variable.set(self.OptionList[0])

        self.servo_variable = tk.StringVar(master)
        self.servo_variable.set(self.ServoOptionList[0])

        tk.Label(self.master, text = "Select Gesture from Menu Below:", font='Helvetica 10 italic').grid(row=0, column=0, columnspan = 3)
        
        self.opt = tk.OptionMenu(self.master, self.variable, *self.OptionList, command = lambda x=self.variable: self.view_gesture(x))
        self.opt.config(width=30, font=('Helvetica', 10))
        self.opt.grid(row=1, column=0, columnspan = 2, padx = button_padx, sticky = 'we')

        self.button_remove = tk.Button(self.master, text = "Delete Gesture", command = lambda: self.delete_gesture())
        self.button_remove.grid(row = 1, column=2, padx = button_padx, sticky = 'we')

        self.view_gesture(self.OptionList[0])

        self.sep = tk.Label(self.master, text = '--------------------------- CHANGE SERVO VALUE BELOW ---------------------------', fg = 'red')
        self.sep.grid(row = 28, column = 0, columnspan = 3, padx = button_padx, sticky = 'we')
        
        self.opt_servo = tk.OptionMenu(self.master, self.servo_variable, *self.ServoOptionList)
        self.opt_servo.config(font=('Helvetica', 10))
        self.opt_servo.grid(row=29, column=0, padx = button_padx, sticky = 'we')

        self.entry_change = tk.Entry(self.master)
        self.entry_change.grid(row = 29, column=1, padx = button_padx, sticky = 'we')

        self.button_change = tk.Button(self.master, text = "Change Value", command = lambda: self.change_value())
        self.button_change.grid(row = 29, column=2, padx = button_padx, sticky = 'we')

    def view_gesture(self, gesture_name):
        self.gesture_name = gesture_name
        gesture = {}
        gesture = self.data[gesture_name]
        i = 3
        for servo, pwm_value in gesture.items():
            if i%2 == 0:
                color = 'blue'
            else:
                color = 'green'
            tk.Label(self.master,fg=color,  text=servo, anchor="w", width = 20, font='Helvetica 10 bold').grid(row=i, column=0, padx = button_padx)
            tk.Label(self.master, fg=color,text=pwm_value, anchor="e", width = 10).grid(row=i, column=2, padx = button_padx)
            i = i + 1

    def change_value(self):
        what_value = self.entry_change.get()
        which_servo = self.servo_variable.get()
        which_gesture = self.variable.get()
        jp.change_object_from_json(filename, which_gesture, which_servo, what_value)
        self.data = jp.read_json(filename)
        self.view_gesture(which_gesture)

    def delete_gesture(self):
        which_gesture = self.variable.get()
        jp.remove_object_from_json(filename, which_gesture)
        self.__init__()

if __name__ == '__main__':
        root = tk.Tk()
        app = Application(master=root)
        app.mainloop()
