import simulator
import serial.tools.list_ports
import tkinter as tk
from tkinter import ttk
import os

class Application(tk.Frame):              
    def __init__(self, master=None):
        tk.Frame.__init__(self, master) 
        self.grid()                     
        self.createWidgets()

    def createWidgets(self):

        #Enter CARLA IP and PORT
        #label
        #Box 1 IP, Box 2 Port


        #For Selecting Mode
        tk.Label(self, text='Select Mode', bd=3).grid(row=9, column=0)
        modes = ["Auto", "Manual"]
        self.modeSelected = tk.StringVar()

        box = ttk.Combobox(self, values=modes, justify="center", textvariable=self.modeSelected)
        box.grid(column=1, row=9, padx=(10), pady=(10))
        box.current(0)

        #Button for connecting elcano to CARLA
        tk.Button(self, text="Connect to CARLA", bg="grey", fg="white", command = self.connectToCarla).grid(column=2, row=9, padx=(10), pady=(10))

        #Button for quit
        tk.Button(self, text='Quit', command=self.quit).grid(row=10, column=1, padx=(10), pady=(10)) 
    
    def connectToCarla(self):
        if self.modeSelected.get() == 'Auto' :
            #pop out for port selection
            win = tk.Toplevel()
            win.wm_title("COM Select")

            tk.Label(win, text='Select COM Port of Routerboard', bd=3).grid(row=0, column=0)

            #Get COM Devices and add them to a dictionary
            devices =  list(serial.tools.list_ports.comports())
            self.deviceDict = {}
            for device in devices:
                self.deviceDict[device.description] = device.device

            #Setup Combobox of Devices
            self.portSelected = tk.StringVar()
            box = ttk.Combobox(win, values=list(self.deviceDict.keys()), justify="center", textvariable=self.portSelected, width=30)
            box.grid(column=0, row=1, padx=(10), pady=(10))
            box.current(0)

            #Button for go
            tk.Button(win, text="GO", bg="grey", fg="white", command = self.startSim).grid(column=0, row=2, padx=(10), pady=(10))


            
        else :
            print("This will start the manual control of carla, use python examples code from carla")


    def startSim(self):
        COMPort = self.deviceDict[self.portSelected.get()]
        simulator.main(COMPort)
        


app = Application()                       
app.master.title('Elcano Project Simulation')    
app.mainloop() 

