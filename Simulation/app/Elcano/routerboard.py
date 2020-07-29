import serial
import time
import threading
import queue
import math
import time



class RouterboardInterface:
    """
    Acts as the interface the routerboard uses to communicate to Carla.

    Using threads, we are able to address all these functions simultaneously
    Reads in commands from buffer and executes them using these headings:
        0x00: self.actuateThrottle,
        0x01: self.actuateSteering,
        0x02: self.actuateBraking,
    
    Also sends data back to router using these headings:
        0x03: Accelerometer Data 
        0x04: Gyro Data         
        0x05: Magnetometer Data          
        0x06: GPS Data       
        0x07: Velocity Data      

    Accepted params:
    COMPort - the COM port the routerboard is plugged into.
    simVehicle - Vehicle in carla to control and read data from
    """


    def __init__(self, COMPort, simVehicle):

        #Class variables
        self.name = "routerboard"
        self.COMPort = COMPort
        self.simVehicle = simVehicle
        self.serialWriteQueue = queue.Queue()   #write queue for the serial output
        self.threadArray = []                   #For all threads this class creates
        self.active = True                      #Flag for threads to see if they should continue running

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

        #Go create the important threads for this controller interface
        self.buildThreads()
        

     



    def buildThreads(self):
        """
        This function builds the thread processes for the class.
        It will create threads, store them in an array and start them.
        """
    

        #Create and append every thread you need tp the threadArray
        self.threadArray.append(threading.Thread(target=self.serialThread))
        self.threadArray.append(threading.Thread(target=self.controlThread))
        self.threadArray.append(threading.Thread(target=self.GPSThread))
        self.threadArray.append(threading.Thread(target=self.compassThread))
        self.threadArray.append(threading.Thread(target=self.velocityThread))
        
        #Start all threads
        for thread in self.threadArray:
            thread.start()





    def controlThread(self):
        """
        Controllers execution, checks for message in serial from router to execute
        """

        while self.active:

            #if command is waiting execute command
            if self.serial.in_waiting:
                    
                #Get the command function, if it doesn't exist, will return None
                command = self.commandMap.get(self.serial.read())

                if command != None:
                    command()


            #Suspend the thread for 0.5 seconds
            time.sleep(0.005)
                
        
    def destroy(self):
        """
        Stop the thread and close serial communication
        """

        #Set flag for threads to false
        self.active = False

        #Wait for all threads to exit
        for thread in self.threadArray:
            thread.join()

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
        self.simVehicle.updateThrottle(carlaValue)


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
        """
        Thread for queuing compass data to send to routerboard
        Collects data from vehicle sensor and queues in the correct format.
        """

        while self.active:

            #Need to send [xhi,xlo,zhi,zlo,yhi,ylo] bytes xx,zz,yy
            #Get data and convert it to bytes, z coordinates is 0 always
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
        Thread for queueing GPS data to send to routerboard
        Collects data from vehicle sensor and queues in the correct format.
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


    def velocityThread(self):
        """
        Thread for queuing velocity data to send to routerboard
        Collects data from vehicle and queues in the correct format.
        """

        while self.active:

            #Get speed from sim (meters/second)
            speed = self.simVehicle.getSpeed()

            #If speed is 0 then just send 4 bytes of FFs
            if speed < 0.001:
                speed = b'\xFF\xFF\xFF\xFF'

            else:
                #We want  μs /pulse
                #1.556 meters(wheel circumference) / velocity * 1000000 = microseconds / pulse
                speed = int(1556000/speed).to_bytes(4, byteorder='big') #convert to 4 bytes

            #Header byte is 7 for velocity data
            message = b'\x07' + speed

            #Write the message to the output queue
            self.serialWriteQueue.put(message)

            #Suspend thread for 0.5 seconds
            time.sleep(0.5)

            



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



# ==============================================================================
# -- Global Functions ----------------------------------------------------------
# ==============================================================================

def mapValue(value, leftMin, leftMax, rightMin, rightMax):
    """
    Function for mapping a value to a given range
    """

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

        