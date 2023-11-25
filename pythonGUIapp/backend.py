import threading
from threading import Thread


class Interface:
    outgoing_messages: list = []
    in_waiting: int = 0
    COM_port: str = None
    baudrate:int = None
    thread: Thread = None
    running: bool = False
    backend_connected = False
    def reset(self):
        self.outgoing_messages.clear()
        self.in_waiting = 0
        self.COM_port= None
        self.thread= None
        self.running = False
        self.backend_connected = False
        
    def _create_backend_thread(self):
        T = threading.Thread(target=backend_mainloop, args=(self,), daemon=True)
        self.thread = T
        return T
    @staticmethod
    def close():
        """will be called by the frontend when exiting the application"""
        pass


    
    @staticmethod
    def on_connected():
        '''notify frontend that connection is established'''
        pass
    @staticmethod
    def on_disconnected():
        '''notify frontend that connection is closed'''
        pass
    @staticmethod
    def on_read( incoming_message: str):
        '''notify frontend that a message is recieved'''
        pass
    @staticmethod
    def on_error( incoming_message: str):
        '''notify frontend that a message is recieved'''
        pass
    @staticmethod
    def on_info( incoming_message: str):
        '''notify frontend that a message is recieved'''
        pass

    
    def disconnect(self):
        if not self.thread:
            return

    
    def connect(self,COM_port,baudrate):
        self.COM_port = COM_port
        self.baudrate = int(baudrate)
        self._create_backend_thread().start()

    def write(self, outgoing_message: str):
        '''send a message from front to the back'''
        if not self.thread:return
        self.outgoing_messages.append(outgoing_message)
        self.in_waiting = len(self.outgoing_messages)

    def _get_message(self):
        '''used by backend to get frontend's pending message'''
        x = self.outgoing_messages.pop(0)
        self.in_waiting = len(self.outgoing_messages)
        return x


def backend_mainloop(interface: Interface):
    import serial
    import time

    # Control Panel\Hardware and Sound\Devices and Printers
    print("Start")
    port = interface.COM_port.strip()  # This will be different for various devices and on windows it will probably be a COM port.
    baudrate = interface.baudrate
    connected = False
    try:
        bluetooth = serial.Serial(
            port, baudrate
        )  # Start communications with the bluetooth unit
        bluetooth.flushInput()  # This gives the bluetooth a little kick
        connected = True
    except:
        interface.on_error("Could not Connect")
        interface.thread = None    
    if not connected :return

    print("Connected")
    interface.on_connected()
    interface.running = True

    def close():
        # print("closing")
        interface.running = False

    interface.close = close
    i = 1
    while interface.running:  # send 5 groups of data to the bluetooth
        try:
            if bluetooth.in_waiting:
                input_data = (
                    bluetooth.readline()
                )  # This reads the incoming data. In this particular example it will be the "Bluetooth answers" line
                # input_str = input_data.decode()
                input_str = input_data.decode('utf-8', errors='ignore')
                
                print(f">> {input_str.strip()}")  # These are bytes coming in so a decode is needed
                interface.on_read(input_str)
            i += 1
            if interface.in_waiting:
                # bluetooth.write(f"yahoo-{i};".encode())
                msg = interface._get_message()
                bluetooth.write(msg.encode())

        except Exception as e:
            print(f'BACKEND_ERROR:')
            print(e)
            interface.running = False
            interface.on_error(e)
            break

        time.sleep(0.001)  # A pause between bursts
    # bluetooth.write(b"LED OFF") #Turn Off the LED, but no answer back from Bluetooth will be printed by python

    bluetooth.close()  # Otherwise the connection will remain open until a timeout which ties up the /dev/thingamabob
    interface.thread = None
    print("Disconnected")
    


def create_backend_thread():
    I = Interface()
    T = threading.Thread(target=backend_mainloop, args=(I,), daemon=True)
    I.thread = T
    return (T, I)


# import serial

# serialPort = serial.Serial(port='COM4', baudrate=9600, timeout=0, parity=serial.PARITY_EVEN, stopbits=1)
# size = 1024

# def SerialSend(string):

#     serialPort.write("yahoo-60;".encode())
# while 1:
#     data = serialPort.readline(size)


#     if data:
#         print(data)
