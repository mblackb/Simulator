
'''

Elcano Carla Simulator Capstone
UW Bothell, 2020

Advisor : Tyler Folsom

Team 2019 : Zach Gale, Jonah Lim, Matthew Moscola, Francisco Navarro-Diaz
Team 2020 : Colton Sellers, Brandon Thompson

simulator.py

Version: 1.0

The main purposes of this program are:
    - Control creation of actors within Carla.
    - Retrieve simulated sensor data from actors and send to router board.
    - Interpret actuation data from router board and send actuation commands to Carla.

Much of the code that prepares the simulation (spawning actors and sensors) can be found in the examples
that come with the CARLA simulator download.

CARLA open-source simulator can be found here: http://Carla.org/



'''

#External imports
import glob
import os
import sys
import serial
import logging
import random
import time

#import Carla and and Sensors
import Carla
import Elcano


#Wait for input before attempting to connect
print("Welcome to the Elcano Project Simulation")
input("Press enter when prepared to connect to server")

#Create the vehicle and connect to simulator
trike = Elcano.Vehicle()
trike.connectToSim('localhost', 2000)

#Create command map for simulator from routerboard
commandMap = {
    0:
        trike.updateThrottle,
    1:
        trike.updateSteering,
    2:
        trike.updateBraking,
}

#Set COM port for router communication and send a starter message
routerboard = serial.Serial('COM10', baudrate = 115200, timeout=5)
time.sleep(1)
routerboard.write('f'.encode('utf-8')) 


try:

    while True:
            # Wait for ready queue from routerboard
            if routerboard.in_waiting:

                #depending on the command get the appropriate function and execute it
                command = commandMap.get(float(routerboard.readline().decode('ASCII')), "nothing")
                arguement = float(routerboard.readline().decode('ASCII'))
                command(arguement)

                ''' 
                ## Receive in order: throttle, steer, brake
                t = float(routerboard.readline().decode('ASCII'))
                s = float(routerboard.readline().decode('ASCII'))
                b = float(routerboard.readline().decode('ASCII'))
                
                trike.actor.apply_control(Carla.VehicleControl(throttle=t,steer=s,brake=b))
                
                Finish processing actuation commands here
                
                Here's how data is sent from Due:
                - Throttle : float (-1 to 1)
                - Steering : float (-1 to 1)
                - Brakes   : float (0 for off 0.3 for on) <- because current implementation of brake
                    is siimply on/off.  Feel free to change on value of 0.3.
                '''
    

except KeyboardInterrupt:
    trike.destroy()
    sys.exit()


