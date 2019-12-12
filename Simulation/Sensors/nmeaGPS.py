'''
Function for GPS sensor in Carla.  Runs every time Carla updates the sensors.  Uses latitude
and longitude position of the vehicle to compute NMEA GPS location.
'''

import time
import math
#import serial
import os,sys,inspect
from data_logger import DataLogger
sys.path.insert(1, os.path.join(sys.path[0], '..'))

def formatFloat(fmt, val):
  ret = fmt % val
  if ret.startswith("0."):
    return ret[1:]
  if ret.startswith("-0."):
    return "-" + ret[2:]
  return ret

def decdeg2dms(dd):
   dd = abs(dd)
   degrees = math.trunc(dd)
   minutes = dd*100000.0
   minutes = minutes - math.trunc(minutes)
   minutes = minutes*100.0
   minutes = formatFloat('%.4f',minutes)
   return (degrees,minutes)


def consoleout(data,logger):
    #print("yes")
    if logger.getListen() is False:
        return
    
    longitude = decdeg2dms(data.longitude)
    latitude = decdeg2dms(data.latitude)
    
    nmea = "$GPGLL,"
    nmea += "{:d}".format(longitude[0])
    nmea += "{:s}".format(longitude[1])
    nmea +=  ",N," 
    nmea += "{:d}".format(latitude[0])
    nmea += "{:s}".format(latitude[1])
    nmea +=  ",W@"
    
    logger.setNmea(nmea)
    
    #dueNative.write(nmea.encode('utf-8'))
    #time.sleep(3)