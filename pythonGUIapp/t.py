import tkinter as tk


def make_dynamic(widget, uniform=False, rows=None, columns=None):
    col_count, row_count = widget.grid_size()
    print(f"{row_count},{col_count}")
    if rows == None:
        rows = range(row_count)
    for i in rows:
        widget.grid_rowconfigure(i, weight=1, uniform=uniform)

    if columns == None:
        columns = range(col_count)
    for i in columns:
        widget.grid_columnconfigure(i, weight=1, uniform=uniform)


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

        content_frame = tk.Frame(root, highlightbackground="blue", highlightthickness=1)
        content_frame.pack(padx=20, pady=20, fill="both")
        for i in range(3):
            slider = tk.Button(content_frame, text="helo")
            slider.grid(row=i, column=0, sticky="EW",padx=(4,0))
        
        def func(sl):
                x = sl
                return lambda event:print(x.get())
        for i in range(3):
            slider = tk.Scale(content_frame, from_=0, to=100, orient="horizontal")
            slider.grid(row=i, column=1,sticky='nsew',pady=(0,8)) 
            slider.bind("<ButtonRelease-1>", func(slider))
        for i in range(3):
            slider = tk.Button(content_frame, text="helo")
            slider.grid(row=i, column=2, sticky="EW",padx=(0,4))
        make_dynamic(content_frame,columns=range(1,2))

        

    def add_label(self, t):
        print(f"adding {t}")


if __name__ == "__main__":
    root = tk.Tk()
    root.minsize(300, 200)
    app = SerialControlPersistApp(root)
    root.mainloop()
