import simulator
import tkinter as tk
import os


def run():
    simulator.main()

def startCarla():
    os.exec

class appWindow:
    
    def __init__(self):

        #Create the window and assets within
        self.window=tk.Tk()
        self.assets = []

        #Assign window info
        self.window.title("Running Python Script")
        self.window.geometry('550x200')

        #Add all the elements
        self.buildWindow()

        



    #Maybe load in elements from different 
    def buildWindow(self):

        #Button for starting CARLA? Need to define Carlas location on a static basis
        btn1 = tk.Button(self.window, text="Start CARLA", bg="grey", fg="white",command=startCarla)
        btn1.grid(column=2, row=0, padx=(100), pady=(100))
        
        self.addAsset(btn1)
        


        #Button for connecting elcano to CARLA
        btn2 = tk.Button(self.window, text="Connect to CARLA", bg="grey", fg="white",command=run)
        btn2.grid(column=5, row=0, padx=(100), pady=(100))

        self.addAsset(btn2)



    #Add asset to list of assets
    def addAsset(self, asset):
        self.assets.append(asset)



    
    #Begin the window
    def startWindow(self):
        #Run the window
        self.window.mainloop()





window = appWindow()
window.startWindow()
