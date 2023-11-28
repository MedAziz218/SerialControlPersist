import tkinter
import tkinter.messagebox
import customtkinter
from datetime import datetime
import serial.tools.list_ports
from backend import create_backend_thread, Interface
from tkinter import filedialog
import os 
import json

def save_to_json(data:dict,initial_dir:str|None =None):
    if not initial_dir: initial_dir = os.getcwd()  # Get the current working directory
    # file_path = filedialog.askopenfilename(initialdir=initial_dir)
    filetypes = [("JSON files", "*.json"), ("All files", "*.*")]
    file_path = filedialog.asksaveasfilename(defaultextension=".json", filetypes=filetypes,initialdir=initial_dir)
    print("Selected file:", file_path)
    if file_path:
        try:
            with open(file_path, 'w') as json_file:
                json.dump(data, json_file)
            print("Data saved to:", file_path)
        except Exception as e:
            print("Error:", e)
    return file_path

def load_from_json(initial_dir:str|None =None):
    # Ask the user to select a JSON file for loading
    if not initial_dir: initial_dir = os.getcwd() 
    filetypes = [("JSON files", "*.json"), ("All files", "*.*")]
    file_path = filedialog.askopenfilename(filetypes=filetypes, initialdir=initial_dir)

    if file_path:
        try:
            with open(file_path, 'r') as json_file:
                data = json.load(json_file)
                print("Data loaded from:", file_path)
                # Do something with the loaded data, for example, print it
                print(data)
        except Exception as e:
            print("Error:", e)
    return data


customtkinter.set_appearance_mode("System")  # Modes: "System" (standard), "Dark", "Light"
customtkinter.set_default_color_theme("blue")  # Themes: "blue" (standard), "green", "dark-blue"
def isfloat(s):
    return s.replace(".", "").strip('-').isnumeric()
def steps_to_numberofsteps(steps,from_,to):
    return int((to - from_) / steps*100)/100

def get_current_timestamp():
    return datetime.now().strftime("%H:%M:%S")
def get_formatted_subsecond_timestamp():
    timestamp = datetime.now()
    formatted_timestamp = timestamp.strftime("%H:%M:%S.%f")[:-3]
    return formatted_timestamp
def center_window(window, width, height):
    screen_width = window.winfo_screenwidth()
    screen_height = window.winfo_screenheight()

    x = (screen_width - width) // 2
    y = (screen_height - height) // 2

    window.geometry(f"{width}x{height}+{x}+{y}")

class ItemSettings:
    def __init__(self,radiobutton,entry,slider,destroy_method=None) -> None:
        self.radiobutton=radiobutton
        self.entry = entry
        self.slider = slider
        self.destroy_method = destroy_method
    def get_cmd(self):
        radiobutton,entry,slider = self.elements()
        key = radiobutton.cget('text')
        val = entry.cget('textvariable').get()
        sep = '-'
        cmd = f'{key}{sep}{val};'
        return cmd
    def destroy(self):
        radiobutton,entry,slider = self.elements()
        entry.destroy()
        slider.destroy()
        radiobutton.destroy()
        if self.destroy_method: self.destroy_method()

    def elements(self):
        return (self.radiobutton,self.entry,self.slider)
    def get(self):
        # settings = {"text":itemtext ,"from":0, "to":100, "number_of_steps":None }
        radiobutton,entry,slider = self.elements()
        settings = {}
        settings["text"] = radiobutton.cget("text")
        settings["from"] = slider.cget("from_")
        settings["to"] = slider.cget("to")
        settings["number_of_steps"] = slider.cget("number_of_steps")
        settings["value"] = entry.cget("textvariable").get()
        return settings
    def set(self,settings:dict):
        # settings = {"text":itemtext ,"from":0, "to":100, "number_of_steps":None }
        radiobutton,entry,slider = self.elements()
           
        if "text" in settings:
            radiobutton.configure(text=settings["text"])
        if "from" in settings:
            slider.configure(from_=float(settings["from"]))
        if "to" in settings:
            slider.configure(to=float(settings["to"]))
        if "step" in settings:
            v = settings["step"]
            if v.lower() == "none" or v =='' or v == '0': v = None
            else : v = steps_to_numberofsteps(float(v),float(settings["from"]),float(settings["to"]))
            print(v)
            slider.stepsize = float(settings["step"]) if v else 0.1
            slider.configure(number_of_steps=v)
        elif "number_of_steps" in settings:
            x = str(settings["number_of_steps"])
            if x and x.lower()!= "none":
                x = float(settings["number_of_steps"])
            else :
                x=None
            slider.configure(number_of_steps=x)
        if "value" in settings:
            entry.cget("textvariable").set(settings['value'])
        return settings
   

class ScrollableRadiobuttonConfigFrame(customtkinter.CTkScrollableFrame):
    def __init__(self, master, command=None, **kwargs):
        super().__init__(master, **kwargs)
        # settings = {"text":itemtext ,"from":0, "to":100, "number_of_steps":None }
        # label = customtkinter.CTkLabel(self, text=item, image=image, compound="left", padx=5, anchor="w")
        self.grid_columnconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=1)

        
        settings_names = ["text","from","to","step"]
        self.settings_names = settings_names
        self.entries = {}
        self.selected_item:ItemSettings = None
        for i in range(len(settings_names)):
            label = customtkinter.CTkLabel(self, text=f"{settings_names[i]} :",compound="left", anchor="w")
            entry_var = customtkinter.StringVar(value="")
            entry = customtkinter.CTkEntry(self, placeholder_text="", textvariable=entry_var)
            label.grid(row=i, column=0, padx=(10, 10), pady=(5, 5), sticky="nsew")
            entry.grid(row=i, column=1, padx=(0, 10), pady=(5, 5), sticky="nsew")
            self.entries[settings_names[i]] = entry_var
        row = len(settings_names)
        self.save_btn = customtkinter.CTkButton(self, command=self.save_selected_settings,text="Save")
        self.save_btn.grid(row=row,column=0,sticky="nsew",pady=(10,0))
        self.save_btn = customtkinter.CTkButton(self, command=self.delete_selected_settings,text="Delete",fg_color="#cc0000",hover_color="#f26868")
        self.save_btn.grid(row=row,column=1,sticky="nsew",pady=(10,0))
        
    def save_selected_settings(self):
        if self.selected_item:
            self.selected_item.set(self.get_entries())
    def delete_selected_settings(self):
        if self.selected_item:
            self.selected_item.destroy()
    def set_selected(self,itemset:ItemSettings):
        self.selected_item = itemset
        self.set_entries(itemset.get())

    def set_entries(self,settings):
        for key in self.settings_names:
            if key in settings:
                self.entries[key].set(settings[key])
            else:
                self.entries[key].set("")

    def get_entries(self):
        return {key:self.entries[key].get() for key in self.settings_names}

    def clear(self):
        for child in self.winfo_children():
            child.destroy()    
class ScrollableRadiobuttonFrame(customtkinter.CTkScrollableFrame):
    send_message = None
    def __init__(self, master, command=None, **kwargs):
        super().__init__(master, **kwargs)
        self.grid_columnconfigure(2, weight=1)
        self.grid_columnconfigure(1, weight=0)
        self.grid_columnconfigure(0, weight=0)
        self.counter = 0
        self.command = command
        self.radiobutton_variable = customtkinter.StringVar()
        
        self.radiobutton_list = {}
        self.selected_radio_button = None
        
    def get(self):
        return self.radiobutton_list
    def radio_button_command(self):
        val = int(self.radiobutton_variable.get())
        self.command(self.radiobutton_list[str(val)])
        print(f">> {self.radiobutton_variable.get()}")
    # def get_item_settings(self,key):
    #     # settings = {"text":itemtext ,"from":0, "to":100, "number_of_steps":None }
    #     radiobutton,entry,slider = self.radiobutton_list[key]
    #     settings = {}
    #     settings["text"] = radiobutton.cget("text")
    #     settings["from"] = slider.cget("from_")
    #     settings["to"] = slider.cget("to")
    #     settings["number_of_steps"] = slider.cget("number_of_steps")
    #     return settings
    
    # def set_item_settings(self,key,settings):
    #     # settings = {"text":itemtext ,"from":0, "to":100, "number_of_steps":None }
    #     radiobutton,entry,slider = self.radiobutton_list[key]
    #     if "text" in settings:
    #         radiobutton.configure(text=settings["text"])
    #     if "from" in settings:
    #         slider.configure(from_=settings["from"])
    #     if "to" in settings:
    #         slider.configure(to=settings["to"])
    #     if "number_of_steps" in settings:
    #         slider.configure(number_of_steps=settings["number_of_steps"])
    #     return settings
    
    def add_item(self,itemsettings:dict=None, itemtext=None, key=None)->ItemSettings:
        if not key:
            key = self.counter
        if key in self.radiobutton_list:
            key = max(list(self.radiobutton_list.keys()))+1
            self.counter = key
        if not itemtext:
            itemtext = f"var_{key}"
            # item = "var"
        row = len(self.radiobutton_list)
        radiobutton = customtkinter.CTkRadioButton(self, text=itemtext, value=key, variable=self.radiobutton_variable,width=50 )
        entry_var = customtkinter.StringVar(value="0")
        entry = customtkinter.CTkEntry(self, placeholder_text="",width=100, textvariable=entry_var)
        sliderVar = tkinter.DoubleVar(value=0)
        slider = customtkinter.CTkSlider(self, from_=0, to=100,number_of_steps=None,variable=sliderVar)
        slider.stepsize = 0.1
        def slider_command(val):
            entry_var.set(round(sliderVar.get(),4))
        def entry_command():
            x = entry_var.get()
            if isfloat(x): sliderVar.set( round(float(x),3) )

        slider.configure(command=slider_command)
        entry_var.trace_add("write",lambda*args:entry_command())
        def inc(*args):
            x = entry_var.get()
            if not isfloat(x):return
            x = float(x)+slider.stepsize
            entry_var.set(round(x,4))
            # entry_command()

        def dec(*args):
            x = entry_var.get()
            if not isfloat(x):return
            x = float(x)-slider.stepsize
            entry_var.set(round(x,4))
            # entry_command()

        def send(*args):
            cmd = setting.get_cmd()
            self.send_message(cmd)
        
        slider.bind("<ButtonRelease-1>",send)
        entry.bind("<Return>",send)
        entry.bind("<Up>",inc)
        entry.bind("<Down>",dec)

        # entry.configure(command=entry_command)    
        
        
        radiobutton.configure(command=self.radio_button_command)
        
        radiobutton.grid(row=row, column=0,padx=(0,5), pady=(5, 5),sticky="w")
        
        entry.grid(row=row, column=1, padx=(0, 0), pady=(5, 5), sticky="nsew")
        slider.grid(row=row, column=2, padx=(0, 10), pady=(10, 10), sticky="nsew" )
        def destroy():
            del self.radiobutton_list[str(key)]

        setting = ItemSettings(radiobutton,entry,slider,destroy_method=destroy)
        self.radiobutton_list[str(key)] = setting
        if itemsettings:
            setting.set(itemsettings)
        
       
        
        #if key == 0:
            #self.set_item_settings(key,{"text":"yahoo","number_of_steps":5})
        self.counter +=1
        return setting
        
    def remove_item(self, item):
        for radiobutton in self.radiobutton_list:
            if item == radiobutton.cget("text"):
                radiobutton.destroy()
                self.radiobutton_list.remove(radiobutton)
                return

    def get_checked_item(self):
        return self.radiobutton_variable.get()

class App(customtkinter.CTk):
    COM_optionemenu_disconnected_color = ("#cc0000","#cc0000")
    COM_optionemenu_connected_color = ("#29A368","#29A368")
    COM_optionemenu_normal_color = ['#3B8ED0', '#1F6AA5']
    backend_interface :Interface = None
    baudrates_list = [
                        "300", "1200", "2400", "4800", "9600", "14400",
                        "19200", "28800", "38400", "57600", "115200", "230400", "250000",
                        "500000", "1000000", "2000000", "2500000", "3000000", "3500000",
                        "4000000"]
    def on_quit(self):
        self.disconnect_callback()
        self.quit()

    def connect_callback(self):
        port = self.COM_optionemenu_var.get()
        baudrate = self.Baudrate_optionemenu_var.get()
        if not port.startswith("COM"):
            return self.show_message("Select COM port first","error")
        
        if self.backend_interface.thread :
            return self.show_message("Disconnect first","error")
        self.show_message(f"Connecting to {port}...","info")
        self.backend_interface.connect(port,baudrate)

    def disconnect_callback(self):    
        if not self.backend_interface.thread:
            return self.show_message("Not Connected","error")
        self.backend_interface.close()

        self.backend_interface.thread.join(1.0)
        self.backend_interface.reset()
        self.backend_interface.on_disconnected()
    def __init__(self):
        super().__init__()
        self.backend_interface = Interface()
        self.protocol("WM_DELETE_WINDOW", self.on_quit)
        # configure window
        self.title("SerialControl")
        # self.geometry(f"{1100}x{600}")  
        center_window(self,1100,600)
        # configure grid layout (4x4)
        
        self.grid_columnconfigure(1, weight=1)
        self.grid_columnconfigure((2, 3), weight=0)
        self.grid_rowconfigure((0, 1, 2), weight=1)
         # create scrollable frame
        self.scrollable_frame = ScrollableRadiobuttonConfigFrame(self,label_text="item config")
        self.scrollable_frame.grid(row=0, column=3,rowspan=2, padx=(20, 0), pady=(10, 0), sticky="nsew")
        
        # create scrollable radiobutton frame
        func = lambda x :self.scrollable_frame.set_selected(x)
        self.scrollable_radiobutton_frame = ScrollableRadiobuttonFrame(master=self, command= func,label_text="items list")
        self.scrollable_radiobutton_frame.send_message = self.send_message
        self.scrollable_radiobutton_frame.grid(row=0, column=1,columnspan=2,rowspan=2,padx=(20,0), pady=(10, 0), sticky="nsew")
        self.scrollable_radiobutton_frame.remove_item("item 3")
        # create sidebar frame with widgets
        self.sidebar_frame = customtkinter.CTkScrollableFrame(self, width=170, corner_radius=0,)
        self.sidebar_frame.grid(row=0, column=0, rowspan=4, sticky="nsew")
        self.sidebar_frame.grid_rowconfigure(4, weight=1)
        self.logo_label = customtkinter.CTkLabel(self.sidebar_frame, text="SerialControl", font=customtkinter.CTkFont(size=20, weight="bold"))
        self.logo_label.grid(row=0, column=0, padx=20, pady=(20, 10))
        #connect button
        self.sidebar_button_1 = customtkinter.CTkButton(self.sidebar_frame,text="Connect")
        self.sidebar_button_1.configure(command = self.connect_callback)
        self.sidebar_button_1.grid(row=1, column=0, padx=20, pady=10)
        #disconnect button
        self.sidebar_button_2 = customtkinter.CTkButton(self.sidebar_frame,text="Disconnect")
        self.sidebar_button_2.configure(command = self.disconnect_callback)
        self.sidebar_button_2.grid(row=2, column=0, padx=20, pady=10)
        
        #empty row here

        #save Profile button
        self.sidebar_button_4 = customtkinter.CTkButton(self.sidebar_frame,text="Save Profile")
        self.sidebar_button_4.configure(command = self.save_profile_callback)
        self.sidebar_button_4.grid(row=5, column=0, padx=20, pady=10)
        #load Profile button
        self.sidebar_button_5 = customtkinter.CTkButton(self.sidebar_frame,text="Load Profile")
        self.sidebar_button_5.configure(command = self.load_profile_callback)
        self.sidebar_button_5.grid(row=4, column=0, padx=20, pady=10)

        #empty row here

        # create COM Option menu
        self.COM_label = customtkinter.CTkLabel(self.sidebar_frame, text="COM port", anchor="w")
        self.COM_label.grid(row=7, column=0, padx=20, pady=(10, 0))
        
        
        self.COM_optionemenu_var = customtkinter.StringVar(value="disconnected")
        self.COM_optionemenu = customtkinter.CTkOptionMenu(self.sidebar_frame, values=["disconnected"],
                                                        command=self.change_COM_event, variable=self.COM_optionemenu_var )
        self.COM_optionemenu.grid(row=8, column=0, padx=20, pady=(10, 0))
        # create COM Option menu
        self.Baudrate_label = customtkinter.CTkLabel(self.sidebar_frame, text="Baudrate port", anchor="w")
        self.Baudrate_label.grid(row=9, column=0, padx=20, pady=(10, 0))
        self.Baudrate_optionemenu_var = customtkinter.StringVar(value="9600")
        self.Baudrate_optionemenu = customtkinter.CTkOptionMenu(self.sidebar_frame, values=self.baudrates_list,
                                                         variable=self.Baudrate_optionemenu_var )
        self.Baudrate_optionemenu.grid(row=10, column=0, padx=20, pady=(10, 0))
        
        s_row = 11
        self.appearance_mode_label = customtkinter.CTkLabel(self.sidebar_frame, text="Appearance Mode:", anchor="w")
        self.appearance_mode_label.grid(row=s_row, column=0, padx=20, pady=(10, 0))
        self.appearance_mode_var = customtkinter.StringVar(value="System")
        self.appearance_mode_optionemenu = customtkinter.CTkOptionMenu(self.sidebar_frame, values=["Light", "Dark", "System"],
                                                                       variable=self.appearance_mode_var,
                                                                       command=self.change_appearance_mode_event)
        
        self.appearance_mode_optionemenu.grid(row=s_row+1, column=0, padx=20, pady=(10, 10))
        self.scaling_label = customtkinter.CTkLabel(self.sidebar_frame, text="UI Scaling:", anchor="w")
        self.scaling_label.grid(row=s_row+2, column=0, padx=20, pady=(0, 0))
        self.scaling_optionemenu = customtkinter.CTkOptionMenu(self.sidebar_frame, values=["80%", "90%", "100%", "110%", "120%"],
                                                               command=self.change_scaling_event)
        self.scaling_optionemenu.grid(row=s_row+3, column=0, padx=20, pady=(10, 20))
        
       
        def update_values(e):
            L = []
             
            available_ports = list(serial.tools.list_ports.comports())
            for port, desc, hwid in available_ports:
                L.append(port)
            if not L:
                self.COM_optionemenu_var.set("no Ports found")
                
            self.COM_optionemenu.configure(values = sorted(L)) 

        self.COM_optionemenu.bind("<ButtonRelease-1>",update_values)
        
        # self.COM_optionemenu.configure(fg_color=self.COM_optionemenu_disconnected_color)
        
        # create textbox
        self.textbox = customtkinter.CTkTextbox(self, width=250)
        self.textbox.grid(row=2, column=1, padx=(20, 0), pady=(20, 0), sticky="nsew")
        custom_font = ("Arial", 16)
        self.textbox.configure(
             font=custom_font, state="disabled" ,wrap="word"
        )

        # set terminal colors and tags
        self.configure_terminal_colors(timestamp="#9E9E9E",outgoing="#A3BDCB",incoming="#2FD34A",info="#ebd13b",error="#cc0000")

        # create main entry and button
        self.entry_var = customtkinter.StringVar(value="yahoo-55;")
        self.entry = customtkinter.CTkEntry(self, placeholder_text="CTkEntry",textvariable=self.entry_var)
        self.entry.grid(row=3, column=1, columnspan=2, padx=(20, 0), pady=(20, 20), sticky="nsew")
       
        self.main_button_1 = customtkinter.CTkButton(master=self,text="send" ,fg_color="transparent", border_width=2, text_color=("gray10", "#DCE4EE"))
        self.main_button_1.grid(row=3, column=3, padx=(20, 20), pady=(20, 20), sticky="nsew")
        def send_cmd():
            self.send_message(self.entry_var.get())

        def on_read(x):
            self.show_message(x.strip(),tag="incoming")

        self.entry.bind("<Return>", lambda event: send_cmd())
        self.main_button_1.configure(command = send_cmd )

        self.backend_interface.on_read = on_read
        self.backend_interface.on_error = lambda e: self.show_message(e,"error")
        self.backend_interface.on_close = lambda e: self.show_message(e)
        self.backend_interface.on_connected = lambda : self.show_message("Connected","info")
        self.backend_interface.on_disconnected = lambda : self.show_message("Disonnected","info")
        # self.backend_interface.on_error = lambda e : self.show_message("Connection Lost","error")

        
        # create actionButtonsframe
        self.actionButtons_frame = customtkinter.CTkScrollableFrame(self,label_text="action buttons")
        self.actionButtons_frame.grid_columnconfigure([0],weight=1)
        self.actionButtons_frame.grid(row=2, column=3, padx=(20, 0), pady=(20, 0), sticky="nsew")
        #add slider button
        self.add_slider_button = customtkinter.CTkButton(self.actionButtons_frame,text="add item")
        self.add_slider_button.configure(command = self.scrollable_radiobutton_frame.add_item)
        self.add_slider_button.grid(row=0, column=0, padx=20, pady=10,sticky='nswe')
        #send all button
        self.clear_items_button = customtkinter.CTkButton(self.actionButtons_frame,text="send all items")
        self.clear_items_button.configure(command = self.send_all_callback)
        self.clear_items_button.grid(row=1, column=0, padx=20, pady=10,sticky='nswe')
        #clear items button
        self.clear_items_button = customtkinter.CTkButton(self.actionButtons_frame,text="clear Items")
        self.clear_items_button.configure(command = self.clear_items_callback)
        self.clear_items_button.grid(row=2, column=0, padx=20, pady=10,sticky='nswe')
        #clear terminal button
        self.clear_terminal_button = customtkinter.CTkButton(self.actionButtons_frame,text="Clear terminal")
        self.clear_terminal_button.configure(command = self.clear_textbox)
        self.clear_terminal_button.grid(row=10, column=0, padx=20, pady=10,sticky='nswe')

        
        # for i in range(100):
        #     switch = customtkinter.CTkSwitch(master=self.scrollable_frame, text=f"CTkSwitch {i}")
        #     switch.grid(row=i, column=0, padx=10, pady=(0, 20))
        #     self.scrollable_frame_switches.append(switch)

        # self.test_button = customtkinter.CTkButton(master=self, fg_color="transparent", border_width=2, text_color=("gray10", "#DCE4EE"))
        # self.test_button.grid(row=0, column=3, padx=(20, 20), pady=(20, 20), sticky="nsew")
    def send_all_callback(self):
        s = self.scrollable_radiobutton_frame.get()
        cmd = ''
        for k in s:
            cmd+=s[k].get_cmd()
        if cmd:
            self.send_message(cmd)

    def clear_items_callback(self):
        s = self.scrollable_radiobutton_frame.get()
        self.scrollable_radiobutton_frame.counter = 0
        keys = list(s.keys())
        for k in keys:
            s[k].destroy()
            
    def save_profile_callback(self):
        data = {"items":[]}
        s = self.scrollable_radiobutton_frame.get()
        for k in s :
            data['items'].append(s[k].get())            
        save_to_json(data)

    def load_profile_callback(self):
        data = load_from_json()
        for item in data['items']:
            self.scrollable_radiobutton_frame.add_item(item)

    def show_message(self,message,tag="info"):
        timestamp = get_formatted_subsecond_timestamp()
        self.textbox.configure(state = "normal")
        self.textbox.insert("end",f'[{timestamp}] -> ',"timestamp")
        self.textbox.insert("end",message+'\n',tag)    
        self.textbox.yview(tkinter.END)
        self.textbox.configure(state = "disabled")  
    def clear_textbox(self):
        self.textbox.configure(state = "normal")
        self.textbox.delete("1.0", "end")
        self.textbox.configure(state = "disabled")  


    def send_message(self,message):
        if not self.backend_interface.running:return
        self.backend_interface.write(message)
        self.show_message(message,"outgoing")

    def configure_terminal_colors(self,**kwargs):
        for k in kwargs:
            self.textbox.tag_config(k, foreground=kwargs[k])
    def change_COM_event(self,val):
        print(f"COM = {val}")
        if self.COM_optionemenu_var.get() != val:
            self.COM_optionemenu_var.set(val)
        


    def sidebar_button_event(self):
        print("sidebar_button click")

    def open_input_dialog_event(self):
        dialog = customtkinter.CTkInputDialog(text="Type in a number:", title="CTkInputDialog")
        print("CTkInputDialog:", dialog.get_input())

    def change_appearance_mode_event(self, new_appearance_mode: str):
        customtkinter.set_appearance_mode(new_appearance_mode)

    def change_scaling_event(self, new_scaling: str):
        new_scaling_float = int(new_scaling.replace("%", "")) / 100
        customtkinter.set_widget_scaling(new_scaling_float)
    def radiobutton_frame_event(self):
        print(f"radiobutton frame modified: {self.scrollable_radiobutton_frame.get_checked_item()}")


if __name__ == "__main__":
    # T,I = create_backend_thread()
    app = App()
    # T.start()
    app.mainloop()