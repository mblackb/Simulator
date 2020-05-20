import serial
import time
import threading
import math



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
            0: self.actuateThrottle,
            1: self.actuateSteering,
            2: self.actuateBraking,
            3: self.getAccelerometer,
        }

        #Connect to routerboard via serial and wait 1 second. Then write a command.
        self.serial = serial.Serial(port=COMPort, baudrate=115200, timeout=5)
        time.sleep(1)
        self.serial.write('f'.encode('utf-8')) 

        #Start the reporting threads, give the system a few seconds first.
        self.GPSThreadTimer = threading.Timer(3.0, self.GPSThread).start()
        #Will add more threading timers for each reported sensor values later


    def execute(self):
        """
        Controllers execution, checks for message in serial from router to execute
        """

        #if command is waiting execute command
        if self.serial.in_waiting:

            #Get the command function, if it doesn't exist, will return None
            command = self.commandMap.get(list(self.serial.read())[0])

            #If its none continue the loop looking for commands
            #Will add some logging later to catch these errors in communication.
            if command == None:
                return
            
            #Execute the command
            command()


    def destroy(self):
        """
        Stop the thread timers and close serial communication
        """
        if self.GPSThreadTimer is not None : self.GPSThreadTimer.cancel()
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
    

    def getAccelerometer (self):

        #EXAMPLE for sending data back to router, doesn't actually
        #do anything yet

        IMU = self.simVehicle.IMUSensor


    def GPSThread(self):
        """
        Timed thread that runs every second to write GPS to serial
        Collects data from vehicle sensor and writes it to serial in the correct format.
        """

        #Create a timed thread that calls this function every 1 seconds
        #By calling this function again it creates another timer endlessly.
        self.GPSThreadTimer = threading.Timer(1.0, self.GPSThread).start()

        #Convert the latitude and longitude to the format we need
        latString = convertDecimaltoMinutesSeconds( self.simVehicle.GNSSSensor.latitude, 'latitude')
        longString = convertDecimaltoMinutesSeconds(self.simVehicle.GNSSSensor.longitude, 'longitude')

        #Header byte is 6 for GPS, reference router.h
        message = "6".encode() + latString.encode() + longString.encode()

        #Writing is also safe as it is blocking and uses a lock!
        self.serial.write(message)


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

        