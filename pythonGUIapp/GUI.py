import tkinter as tk
from tkinter import simpledialog
from itertools import cycle


def get_widget_position_and_height(widget):
    x = widget.winfo_x()
    y = widget.winfo_y()
    height = widget.winfo_height()
    width = widget.winfo_width()

    return (x, y, width, height)

def center_window_on_screen(window, width, height):
    screen_width = window.winfo_screenwidth()
    screen_height = window.winfo_screenheight()
    x = (screen_width - width) // 2
    y = (screen_height - height) // 2
    window.geometry(f"{width}x{height}+{x}+{y}")

def center_window_on_root(root,window, width, height):
    root_width = root.winfo_x()
    root_height = root.winfo_y()
    x = (root_width - width) // 2
    y = (root_height - height) // 2
    window.geometry(f"{width}x{height}+{x}+{y}")


# class ConsoleWidget():
#     def __init__(self, parent, master=None):
#         self.parent = parent
#         self.txt = tk.StringVar()
#         self.rootEntry = tk.Entry(self.parent, textvariable=self.txt)
#         self.rootEntry.pack()

#         self.rootEntry.bind("<Return>", self.onEnter)
#         self.rootText = tk.Text(self.parent)
#         self.rootText.pack()
#         # self.rootText.bind("<Insert>", self.insert_all)
#         # self.newList = []

#     def onEnter(self, arg=None):
#         t = self.textiter.next()
#         self.txt.set("")
#         self.rootText.insert("end", t+"\n")
#         self.newList.append(self.rootText.get("end - 2 chars linestart", "end - 1 chars"))


class ConsoleWidget:
    def __init__(self, root) -> None:
        self.root = root

        self.txt = tk.StringVar()
        self.rootEntry = tk.Entry(self.root, textvariable=self.txt)
        self.rootEntry.pack()
        self.rootEntry.bind("<Return>", self.onReturn)

    def onReturn(self, argg=None):
        print("it works")


class SerialControlPersistApp:
    def __init__(self, root) -> None:
        self.root = root
        self.root.title("Data Input App")

        self.add_label_button = tk.Button(
            root,
            text="Add Label"
            # , command=self.select_label_type
        )
        self.add_label_button.pack()

        add_label = lambda t: self.add_label(t)

        # Create a menu
        menu = tk.Menu(root, tearoff=0)
        menu.add_command(label="Slider", command=lambda: add_label("slider"))
        menu.add_command(label="TextInput", command=lambda: add_label("textinput"))
        menu.add_separator()
        menu.add_command(label="Exit", command=root.quit)

        # Bind the menu to the button
        show_dropdown = lambda event: menu.post(event.x_root, event.y_root)
        self.add_label_button.bind("<ButtonRelease-1>", show_dropdown)

    def add_label(self):
        print("this works too")
        
    def create_string_input_dialog(self):
        def on_ok_button_click():
            result.set(entry.get())
            dialog.destroy()

        dialog = tk.Toplevel()
        dialog.title("String Input Dialog")

        result = tk.StringVar()
        result.set("")  # Initialize the result variable

        label = tk.Label(dialog, text="Enter a string:")
        label.pack(padx=10, pady=10)

        entry = tk.Entry(dialog, textvariable=result)
        entry.pack(padx=10, pady=10)

        ok_button = tk.Button(dialog, text="OK", command=on_ok_button_click)
        ok_button.pack(padx=10, pady=10)

        # Center the dialog on the screen
        # center_window_on_root(self.root,dialog, 300, 150)
        x,y = self.root_center()
        dialog.geometry(f"{300}x{150}+{x}+{y}")
        dialog.mainloop()  # Run the dialog window

        return result.get()
    def add_label(self, label_type):
        assert label_type in ("slider", "textinput")
        # cx,cy = self.root_center()
        label_name = simpledialog.askstring("Label Name", "Enter label name:")
        # label_name = self.create_string_input_dialog()
        if label_name:
            # label_type = self.select_label_type()
            if label_type:
                self.create_label_row(label_name, label_type)

    def root_center(self):
        # Calculate the center position relative to the main window
        x1, y1, w1, h1 = get_widget_position_and_height(self.add_label_button)
        x = (
            self.root.winfo_x()
            + x1
            + w1 // 2
            # + (self.root.winfo_width() - self.label_type_window.winfo_reqwidth()) // 2
        )
        y = (
            self.root.winfo_y()
            + y1
            + h1 * 2
            # + (self.root.winfo_height() - self.label_type_window.winfo_reqheight()) // 2
        )
        return x, y


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

        # self.data.append((label_name, label_type, frame))
        frame.pack()


# class DataInputApp:
#     def __init__(self, root):
#         self.root = root
#         self.root.title("Data Input App")
#         self.data = []

#         self.add_label_button = tk.Button(root, text="Add Label", command=self.add_label)
#         self.add_label_button.pack()

#     def add_label(self):
#         label_name = simpledialog.askstring("Label Name", "Enter label name:")
#         if label_name:
#             label_type = self.select_label_type()
#             if label_type:
#                 self.create_label_row(label_name, label_type)

#     def select_label_type(self):
#         label_type = ""

#         def set_label_type_to_slider():
#             nonlocal label_type
#             label_type = "slider"
#             self.label_type_window.destroy()

#         def set_label_type_to_textinput():
#             nonlocal label_type
#             label_type = "textinput"
#             self.label_type_window.destroy()

#         self.label_type_window = tk.Toplevel(self.root)
#         self.label_type_window.title("Select Label Type")

#         # Calculate the center position relative to the main window
#         x = self.root.winfo_x() + (self.root.winfo_width() - self.label_type_window.winfo_reqwidth()) // 2
#         y = self.root.winfo_y() + (self.root.winfo_height() - self.label_type_window.winfo_reqheight()) // 2
#         self.label_type_window.geometry(f"+{x}+{y}")

#         slider_button = tk.Button(self.label_type_window, text="Slider", command=set_label_type_to_slider)
#         slider_button.pack()

#         text_input_button = tk.Button(self.label_type_window, text="Text Input", command=set_label_type_to_textinput)
#         text_input_button.pack()

#         self.label_type_window.wait_window()

#         return label_type

#     def create_label_row(self, label_name, label_type):
#         frame = tk.Frame(self.root)
#         label = tk.Label(frame, text=label_name)
#         label.grid(row=0, column=0)

#         if label_type.lower() == "slider":
#             slider = tk.Scale(frame, from_=0, to=100, orient="horizontal")
#             slider.grid(row=0, column=1)
#         elif label_type.lower() == "textinput":
#             text_input = tk.Entry(frame)
#             text_input.grid(row=0, column=1)

#         self.data.append((label_name, label_type, frame))
#         frame.pack()
if __name__ == "__main__":
    root = tk.Tk()
    root.minsize(300, 200)
    app = SerialControlPersistApp(root)
    root.mainloop()
