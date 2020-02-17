
'''

Elcano Carla Simulator Capstone
UW Bothell, 2020

Advisor : Tyler Folsom

Team 1 : Zach Gale, Jonah Lim, Matthew Moscola, Francisco Navarro-Diaz
Team 2 : Colton Sellers, Brandon Thompson

simulator.py

Version: 1.0

The main purposes of this program are:
    - Control creation of actors within Carla.
    - Retrieve simulated sensor data from actors and send to router board.
    - Interpret actuation data from router board and send actuation commands to Carla.

Much of the code that prepares the simulation (spawning actors and sensors) can be found in the examples
that come with the CARLA simulator download.

CARLA open-source simulator can be found here: http://Carla.org/

BEFORE RUNNING : 
** Change the default IP address to the address of the PC running Carla.
** Change COM used (USB port) to the one connected with the router board.

'''

#External imports
import glob
import os
import sys
import serial
import logging
import random
import time
import math
from data_logger import DataLogger

#import Carla and and Sensors
import Carla
import Sensors
import Elcano


#Wait for input before attempting to connect
print("Welcome to the Elcano Project Simulation")
input("Press enter when prepared to connect to server")

#Create the vehicle and connect to simulator and routerboard
trike = Elcano.Vehicle()
trike.connectToSim()
#trike.connectToRouter()



try:
    
    trike.demo()

except KeyboardInterrupt:
    trike.destroy()
    sys.exit()


