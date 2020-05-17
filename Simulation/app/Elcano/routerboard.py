import serial
import time

class RouterboardInterface:

    #Constructor, requires an obj for simulate
    def __init__(self):
        self.name = "router"


    #Connect to the router board
    def connectToBoard(self, COMport, baudrate, timeout):
        self.serial = serial.Serial(port=COMport, baudrate=baudrate, timeout=timeout)
        time.sleep(1)
        self.serial.write('f'.encode('utf-8')) 

    
    #Command from router to change throttle
    def actuateThrottle(self, simVehicle) :

        #Get the argument from the router board and convert into a Carla value
        #Value comes in as a 8 bit unsigned int (0 to 255)
        arg = list(self.serial.read(1))[0]
        carlaValue = mapValue(arg, 0, 255, 0, 1)

        #Throttle : float (-1 to 1) 
        #No reverse option for now
        simVehicle.updateThrottle(arg)


    #Command from router to change steering
    def actuateSteering(self, simVehicle) :

        #Get the argument from the router board and convert into a Carla value
        #Value comes in as a 8 bit signed int (-128 to 127)
        arg = list(self.serial.read(1))[0]
        carlaValue = mapValue(arg, 0, 255, -1, 1)

        #Steering : flaot (-1 to 1)
        simVehicle.updateSteering(carlaValue)


    #Command from router to change braking
    def actuateBraking(self, simVehicle) :

        #Get the argument from the router board and convert into a Carla value
        #Value comes in as a 8 bit unsigned int (0 to 255)
        arg = list(self.serial.read(1))[0]
        carlaValue = mapValue(arg, 0, 255, 0, 1)

        #Braking : float (0 to 1)
        simVehicle.updateBraking(carlaValue)
    

    def getAccelerometer (self, simVehicle):

        #EXAMPLE for sending data back to router, doesn't actually
        #do anything yet
        self.serial.write(simVehicle.Accelerometer)

    def getGPS(self, simVehicle):

        lat = simVehicle.GPS.latitude
        lon = simVehicle.GPS.longitude

        #write the messages, need to adapt format.
        self.serial.write(6)
        self.serial.write(lat)
        self.serial.write(lon)

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