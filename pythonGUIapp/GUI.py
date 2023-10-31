import tkinter as tk
from tkinter import simpledialog
from itertools import cycle
 
class ConsoleWidget():
    def __init__(self, parent, master=None):
        self.parent = parent
        self.txt = tk.StringVar()
        self.rootEntry = tk.Entry(self.parent, textvariable=self.txt)
        self.rootEntry.pack()

        self.rootEntry.bind("<Return>", self.onEnter)
        self.rootText = tk.Text(self.parent)
        self.rootText.pack()
        # self.rootText.bind("<Insert>", self.insert_all)
        # self.newList = []
 
    def onEnter(self, arg=None):
        t = self.textiter.next()
        self.txt.set("")
        self.rootText.insert("end", t+"\n")
        self.newList.append(self.rootText.get("end - 2 chars linestart", "end - 1 chars"))
 
    
 
class DataInputApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Data Input App")
        self.data = []

        self.add_label_button = tk.Button(root, text="Add Label", command=self.add_label)
        self.add_label_button.pack()

    def add_label(self):
        label_name = simpledialog.askstring("Label Name", "Enter label name:")
        if label_name:
            label_type = self.select_label_type()
            if label_type:
                self.create_label_row(label_name, label_type)

    def select_label_type(self):
        label_type = ""

        def set_label_type_to_slider():
            nonlocal label_type
            label_type = "slider"
            self.label_type_window.destroy()

        def set_label_type_to_textinput():
            nonlocal label_type
            label_type = "textinput"
            self.label_type_window.destroy()

        self.label_type_window = tk.Toplevel(self.root)
        self.label_type_window.title("Select Label Type")
        
        # Calculate the center position relative to the main window
        x = self.root.winfo_x() + (self.root.winfo_width() - self.label_type_window.winfo_reqwidth()) // 2
        y = self.root.winfo_y() + (self.root.winfo_height() - self.label_type_window.winfo_reqheight()) // 2
        self.label_type_window.geometry(f"+{x}+{y}")

        slider_button = tk.Button(self.label_type_window, text="Slider", command=set_label_type_to_slider)
        slider_button.pack()

        text_input_button = tk.Button(self.label_type_window, text="Text Input", command=set_label_type_to_textinput)
        text_input_button.pack()

        self.label_type_window.wait_window()

        return label_type

    def create_label_row(self, label_name, label_type):
        frame = tk.Frame(self.root)
        label = tk.Label(frame, text=label_name)
        label.grid(row=0, column=0)

        if label_type.lower() == "slider":
            slider = tk.Scale(frame, from_=0, to=100, orient="horizontal")
            slider.grid(row=0, column=1)
        elif label_type.lower() == "textinput":
            text_input = tk.Entry(frame)
            text_input.grid(row=0, column=1)

        self.data.append((label_name, label_type, frame))
        frame.pack()
if __name__ == "__main__":
    root = tk.Tk()
    root.minsize(300, 200)
    app = DataInputApp(root)
    root.mainloop()
