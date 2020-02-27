import tkinter as tk
import json_parsing as jp

filename = 'gestures.json'

button_padx = 5
button_pady = 5

class Application(tk.Frame):
    def __init__(self, master = None):
        tk.Frame.__init__(self, master)

        self.PoseDict = {}
        self.OptionList = []

        self.PoseObj = {}

        self.data = jp.read_json(filename)

        self.current_row = 2

        self.delete = False

        for gesture in self.data:
            self.OptionList.append(gesture)

        self.variable = tk.StringVar(master)
        self.variable.set(self.OptionList[0])

        self.entry_save = tk.Entry(self.master)
        self.entry_save.grid(row = 0, column =0, columnspan = 2, sticky = 'we',padx = button_padx)

        self.button_save = tk.Button(self.master, text = 'SAVE', command = lambda: self.save_animation())
        self.button_save.grid(row = 0, column = 2,padx = button_padx)
        
        self.opt = tk.OptionMenu(self.master, self.variable, *self.OptionList)
        self.opt.grid(row=1, column=0,padx = button_padx)

        self.entry_time = tk.Entry(self.master)
        self.entry_time.grid(row = 1, column = 1)

        self.button_add = tk.Button(self.master, text = 'ADD', command = lambda: self.add_pose())
        self.button_add.grid(row = 1, column = 2)

    def add_pose(self):
        tk.Label(self.master, text = self.variable.get()).grid(row = self.current_row, column = 0)
        tk.Label(self.master, text = self.entry_time.get()).grid(row = self.current_row, column = 1)
        self.PoseDict[(self.current_row-1)] = [self.variable.get(), self.entry_time.get()]
        self.current_row = self.current_row + 1

    def save_animation(self):
        jp.add_object_to_json('animations.json', self.entry_save.get(), self.PoseDict)
        jp.pretty_print('animations.json')
        self.__init__()


if __name__ == '__main__':
        root = tk.Tk()
        app = Application(master=root)
        app.mainloop()

