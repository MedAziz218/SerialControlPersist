import tkinter as tk


def do_something():
    print("Button clicked")


def show_dropdown(event):
    print("me is here")
    menu.post(event.x_root, event.y_root)
    # button.config(relief=tk.RAISED)
    print("me is thhhere")


root = tk.Tk()
root.title("Dropdown Menu Example")

# Create a button
button = tk.Button(root, text="Click Me")
button.pack()

def doCommand(f=None):
    if f : f()
# Create a menu
menu = tk.Menu(root, tearoff=0)
menu.add_command(label="Option 1", command=doCommand(lambda:print("Option 1 selected") ))
menu.add_command(label="Option 2", command=doCommand( lambda: print("Option 2 selected")))
menu.add_separator()
menu.add_command(label="Exit", command=root.quit)

# Bind the menu to the button
button.bind("<ButtonRelease-1>", show_dropdown)

def on_enter(event):
    button.config(relief=tk.RAISED)  # Set the button relief style when hovered

def on_leave(event):
    button.config(relief=tk.RAISED)  # Reset the button relief style when not hovered


root.mainloop()
