'''
 Holds the most current trike sensor data from Carla to send to Due.
 Sends last updated data when prompted.

 NOT CURRENTLY USED
 Was apart of the old groups code, not sure if we are gonna use it at all again
 only keeping until sure.
 '''

import serial

class DataLogger:

    def __init__(self, actorId):
        self.actorId = actorId
    
    def setNmea(self, nmea):
        self.nmea = nmea
    
    # seconds per pulse as a string
    def setCyclometer(self, pulse):
        self.pulse = pulse
        
    def setListen(self):
        self.listen = True
        
    def setStopListen(self):
        self.listen = False
        
    def getListen(self):
        return self.listen
        
    def sendNmea(self, ser):
        ser.write(self.nmea.encode('utf-8'))
    
    def sendCyclometer(self, ser):
        ser.write(self.pulse.encode('utf-8'))