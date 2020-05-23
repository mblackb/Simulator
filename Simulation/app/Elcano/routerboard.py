import serial
import time
import threading
import queue
import math
import time



class RouterboardInterface:
    """
    Acts as the interface the routerboard uses to communicate to Carla.

    Reads in commands from buffer and executes them, also sends data back to router.

    Accepted params:
    COMPort - the COM port the routerboard is plugged into.
    """


    def __init__(self, COMPort, simVehicle):
        self.name = "routerboard"
        self.COMPort = COMPort
        self.simVehicle = simVehicle

        #Create command map for incoming commands from routerboard
        self.commandMap = {
            b'\00': self.actuateThrottle,
            b'\01': self.actuateSteering,
            b'\02': self.actuateBraking,
        }

        #Connect to routerboard via serial and wait 1 second. Then write a command.
        self.serial = serial.Serial(port=COMPort, baudrate=115200, timeout=5)
        time.sleep(1)
        self.serial.write('f'.encode('utf-8')) 
        
        #Currently running the controller, used as a flag for threads!!!
        self.active = True

        #Build a write queue and start a thread to manage it
        self.serialWriteQueue = queue.Queue()
        self.serialThread = threading.Thread(target=self.serialThread)
        self.serialThread.start()

        #Create the threads for reporting data and start them
        self.GPSThread = threading.Thread(target=self.GPSThread)
        self.compassThread = threading.Thread(target=self.compassThread)

        self.GPSThread.start()
        self.compassThread.start()


    def execute(self):
        """
        Controllers execution, checks for message in serial from router to execute
        """
        
        #if command is waiting execute command
        if self.serial.in_waiting:
                
            #Get the command function, if it doesn't exist, will return None
            command = self.commandMap.get(self.serial.read())

            if command != None:
                command()
                
        
    def destroy(self):
        """
        Stop the thread timers and close serial communication
        """

        #Set flag for threads to false
        self.active = False

        #Wait for all threads to exit
        self.serialThread.join()
        self.GPSThread.join()
        self.compassThread.join()

        #Close the serial
        self.serial.close()


    def actuateThrottle(self) :
        """
        Command to update throttle on vehicle.
        Reads in the next byte from serial to determine this value.
        """

        #Get the argument from the router board and convert into a Carla value
        #Value comes in as a 8 bit unsigned int (0 to 255)
        arg = list(self.serial.read(1))[0]
        carlaValue = mapValue(arg, 0, 255, 0, 1)

        #Throttle : float (-1 to 1) 
        #No reverse option for now
        self.simVehicle.updateThrottle(arg)


    def actuateSteering(self) :
        """
        Command to update steering on vehicle.
        Reads in the next byte from serial to determine this value.
        """

        #Get the argument from the router board and convert into a Carla value
        #Value comes in as a 8 bit signed int (-128 to 127)
        arg = list(self.serial.read(1))[0]
        carlaValue = mapValue(arg, 0, 255, -1, 1)

        #Steering : flaot (-1 to 1)
        self.simVehicle.updateSteering(carlaValue)


    def actuateBraking(self) :
        """
        Command to update braking on vehicle.
        Reads in the next byte from serial to determine this value.
        """

        #Get the argument from the router board and convert into a Carla value
        #Value comes in as a 8 bit unsigned int (0 to 255)
        arg = list(self.serial.read(1))[0]
        carlaValue = mapValue(arg, 0, 255, 0, 1)

        #Braking : float (0 to 1)
        self.simVehicle.updateBraking(carlaValue)
    
    
    def compassThread(self):
        while self.active:
            #Need to send [xhi,xlo,zhi,zlo,yhi,ylo] bytes xx,zz,yy
            angle = self.simVehicle.IMUSensor.radiansCompass
            dataX = (int(math.cos(angle)*32767)).to_bytes(2, byteorder='big', signed ='true')
            dataZ = b'\x00\x00'
            dataY = (int(math.sin(angle)*32767)).to_bytes(2, byteorder='big', signed ='true')

            #Header for message is x05
            message = b'\x05' + dataX + dataZ + dataY

            #Write the message to the output queue
            self.serialWriteQueue.put(message)

            #Suspend the thread for 0.5 seconds
            time.sleep(0.5)


    def GPSThread(self):
        """
        Timed thread that runs every second to write GPS to serial
        Collects data from vehicle sensor and writes it to serial in the correct format.
        """

        while self.active:

            #Convert the latitude and longitude to the format we need
            latString = convertDecimaltoMinutesSeconds(self.simVehicle.GNSSSensor.latitude, 'latitude')
            longString = convertDecimaltoMinutesSeconds(self.simVehicle.GNSSSensor.longitude, 'longitude')

            #Header byte is 6 for GPS, reference router.h
            message = b'\x06' + latString.encode() + longString.encode()

            #Write the message to the output queue
            self.serialWriteQueue.put(message)

            #Suspend thread for 1.0 seconds
            time.sleep(1.0)


    def serialThread(self):
        """
        Thread for managing the outward messages.
        Necessary to handle all the various sensor messages
        """

        while self.active:

            #Write all the messages in the queue
            while not self.serialWriteQueue.empty():
                self.serial.write(self.serialWriteQueue.get())
            
            #Once queue is empty just suspend for a short time
            time.sleep(0.005)


def mapValue(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)

def convertDecimaltoMinutesSeconds(val, convertType):
    """
    For converting degrees decimal (dd.dddd) to degrees minutes seconds lat (ddmm.ssss) long (dddmm.ssss)
    also returns the directional, for lat N|S, for long E|W     (+|-)

    Format we are looking for:
    lat[9 bytes], latdir,  (ddmm.ssss N|S) ex: 4559.4810N
    long[10bytes], longdir (dddmm.ssss E|W) ex: 12269.3800W
    """

    ddecimal, degrees = math.modf(val)
    mdecimal, minutes = math.modf(ddecimal*60)
    seconds = mdecimal*60

    #Depending on whether its long or lat we determine the proper direction
    if(convertType == 'latitude'): 
        if degrees >= 0 : direction = "N"
        else: direction = "S"

        #Latitude always reports 2 digits
        degreeLength = 2

    elif(convertType == 'longitude'): 
        if degrees >= 0 : direction = "E"
        else : direction = "W"

        #Longitude always reports 3 digits
        degreeLength = 3
    
    #Convert to absolute value string and pad with 0's as necessary
    degrees = str(abs(int(degrees)))
    while(len(degrees) < degreeLength):
        degrees = '0' + degrees

    #Ensure minutes is two digits!
    minutes = str(int(minutes))
    while(len(minutes) < 2):
        minutes = '0' + minutes

    #Remove decimal and ensure it is 4 digits
    seconds = str(seconds).replace('.', '')
    if(len(seconds) < 4):
        while(len(seconds) < 4):
            seconds = seconds + '0'
    else:
        seconds = seconds[0:4]

    string = degrees + minutes + '.' + seconds + direction

    return string

        