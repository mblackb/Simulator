import serial
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
            0: self.actuateThrottle,
            1: self.actuateSteering,
            2: self.actuateBraking,
            3: self.getAccelerometer,
        }

        self.connectToBoard(COMPort, baudrate = 115200, timeout=5)


    def connectToBoard(self, COMport, baudrate, timeout):
        """
        Actually connects to the router board via serial.

        Accepted params:
        COMPort - the COM port the routerboard is plugged into.
        baudrate -
        timeout - Max time it will wait to establish communication over serial.
        """

        self.serial = serial.Serial(port=COMport, baudrate=baudrate, timeout=timeout)
        time.sleep(1)
        self.serial.write('f'.encode('utf-8')) 


    def controlLoop(self):
        """
        Enters the control loop which will handle the commands and sending of data
        """

        #If there is a command waiting
        if self.serial.in_waiting:

            #Get the command function, if it doesn't exist, will return None
            command = self.commandMap.get(list(self.serial.read())[0])

            #If its none continue the loop looking for commands
            #Will add some logging later to catch these errors in communication.
            if command == None:
                return
            
            #Execute the command
            command()

        
        #Send new data across, maybe limit to once a second?
        self.getGPS()

    
    
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


    def getGPS(self):
        """
        Function to send gps data back to router board.
        Collects data from vehicle sensor and writes it to serial in the correct format.
        """

        lat = self.simVehicle.GNSSSensor.latitude
        lon = self.simVehicle.GNSSSensor.longitude

        #write the messages, need to adapt format.
        #self.serial.write(6)
        #self.serial.write(lat)
        #self.serial.write(lon)

        ##Header byte is 6 for GPS, reference router.h
        ##LAT, LONG, ALT, HH:MM:SS.sss once a second, maybe a thread?
        ##lat[9],latdir,long[10],longdir (ddmm.mmmmN/Sdddmm.mmmmE/W) i.e: 4559.4810N12269.3800W
        ##dd is degrees mm.mmmm is minutes ss is seconds


def mapValue(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)