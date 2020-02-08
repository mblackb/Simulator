
'''

Elcano Carla Simulator Capstone
UW Bothell, 2019

Advisor : Tyler Folsom

Team : Team: Zach Gale, Jonah Lim, Matthew Moscola, Francisco Navarro-Diaz

simulator.py

Version: 1.0

The main purposes of this program are:
    - Control creation of actors within Carla.
    - Retrieve simulated sensor data from actors and send to router board.
    - Interpret actuation data from router board and send actuation commands to Carla.

Much of the code that prepares the simulation (spawning actors and sensors) can be found in the examples
that come with the CARLA simulator download.

CARLA open-source simulator can be found here: http://carla.org/

BEFORE RUNNING : 
** Change the default IP address to the address of the PC running Carla.
** Change COM used (USB port) to the one connected with the router board.

'''

#External imports
import glob
import os
import sys
import serial
import argparse
import logging
import random
import time
import math
from data_logger import DataLogger

#import Carla module from .egg file
sys.path.append('./carla/carla-0.9.5.egg')
import carla

#Import our sensors
sys.path.append('./Sensors')
import nmeaGPS
import numGPS


#Arguments accepted for the script --host, --port, --safe
argparser = argparse.ArgumentParser(
    description=__doc__)
argparser.add_argument(
    '--host',
    metavar='H',
    default='localhost',
    help='IP of the host server (default: 127.0.0.1)')
argparser.add_argument(
    '-p', '--port',
    metavar='P',
    default=2000,
    type=int,
    help='TCP port to listen to (default: 2000)')
argparser.add_argument(
    '--safe',
    action='store_true',
    help='avoid spawning vehicles prone to accidents')
args = argparser.parse_args()


#Carla vehicle options
vehiclesOpt = ["trike"]
vehicle = None  # actor ID of the spawned vehicle
sensorsOptDisp = ["nmeaGPS","rotSensor","numGPS"]
sensorsOpt = ["nmeagps","rotsensor","numgps"]
sensors = []

#Data Logger logging functions
logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

#Wait for input before attempting to connect
print("Welcome to the Elcano Project Simulation")
input("Press enter when prepared to connect to server")

#Attempt to connect to the carla server, timeout after 5 seconds
client = carla.Client(args.host, args.port)
client.set_timeout(5.0)

#Get the world from the server
world = client.get_world()

#Enable headless mode
#settings = world.get_settings()
#settings.no_rendering_mode = True
#world.apply_settings(settings)

#Load Carlas blueprint library
blueprint_library = world.get_blueprint_library()

#Grab all the car blueprints from library
carBlueprints = world.get_blueprint_library().filter('vehicle.*')
if args.safe:
    carBlueprints = [x for x in carBlueprints if int(x.get_attribute('number_of_wheels')) == 4]
    carBlueprints = [x for x in carBlueprints if not x.id.endswith('isetta')]
    carBlueprints = [x for x in carBlueprints if not x.id.endswith('carlacola')]

#Gather possible spawn points
spawn_points = world.get_map().get_spawn_points()
number_of_spawn_points = len(spawn_points)

# @todo cannot import these directly.
SpawnActor = carla.command.SpawnActor
SetAutopilot = carla.command.SetAutopilot
FutureActor = carla.command.FutureActor

#Set COM port for router communication
ser = serial.Serial('COM4',baudrate = 115200,timeout=1)

#CLI when starting script
def main():
    print("Welcome to the Carla Simulator Framework")
    while True:
        # print("Current Vehicles:")
        # print("Current Sensors")        
        opt = input("Press s to start simulation, q to quit: ")
        opt = opt.lower()
        if(opt == "q"):
            break
        elif(opt == "s"):
            runSim()    


#Spawns trike at random spawnpoint, adds sensors, starts listening and sending data to router board
def runSim():
    print('sim started')
    
    # Spawn a trike
    batch = []
    blueprint = random.choice(carBlueprints)
    if blueprint.has_attribute('color'):
        color = random.choice(blueprint.get_attribute('color').recommended_values)
        blueprint.set_attribute('color', color)
    blueprint.set_attribute('role_name', 'autopilot')

    random.shuffle(spawn_points)
    batch.append(SpawnActor(blueprint, spawn_points[0]))

    for response in client.apply_batch_sync(batch):
        if response.error:
            logging.error(response.error)
        else:
            vehicle = response.actor_id


    # Attach nmeaSensor to trike (speed directly taken from trike actor)
    actor = world.get_actors().find(vehicle)
    for x in world.get_actors():
        if vehicle == x.id:
            actor = x
    #print(vehicle)
    #print(world.get_actors())
    #next((x for x in world.get_actors() if x.id == vehicle), None)
    
    logger = DataLogger(actor)
    blueprint = world.get_blueprint_library().find('sensor.other.gnss')
    transform = carla.Transform(carla.Location(x=0.8, z=1.7))
    
    nmeaSensor = world.spawn_actor(blueprint,transform,actor)
    sensors.append(nmeaSensor)
    #speedSensor = world.spawn_actor(blueprint,transform,actor)
    
    ### SET SIM VIEWPOINT TO VEHICLE ####
    world.get_spectator().set_transform(actor.get_transform())
    #################
    
    ## RUN SIM ##
    try:
        ser.write('f'.encode('utf-8'))  #Notify Due that system is starting
        
        ##### FOR GPS SENSOR USAGE ####
        logger.setListen()
        logger.setNmea('Test@')  # Fill private member to avoid error
        nmeaSensor.listen(lambda data: nmeaGPS.consoleout(data,logger))
        ##########################
        
        while True:
            # Wait for ready queue from due
            if ser.in_waiting:
                ### FOR GPS SENSOR USAGE (Set the stop variable to short the listener func) ###
                logger.setStopListen()
                #nmeaSensor.stop()
                ###########
                
                ''' 
                ADD SERIAL READS FOR ACTUATION HERE 
                
                Currently just clears an arbitrary char in buffer sent from
                router Due
                '''

                ## Receive in order: throttle, steer, brake
                t = float(ser.readline().decode('ASCII'))
                s = float(ser.readline().decode('ASCII'))
                b = float(ser.readline().decode('ASCII'))
                
                actor.apply_control(carla.VehicleControl(throttle=t,steer=s,brake=b))

                ''' 
                Finish processing actuation commands here
                
                Here's how data is sent from Due:
                - Throttle : float (-1 to 1)
                - Steering : float (-1 to 1)
                - Brakes   : float (0 for off 0.3 for on) <- because current implementation of brake
                    is siimply on/off.  Feel free to change on value of 0.3.
                '''
                
                ### ACCESS/SEND LAT/LONG FROM LAST TICK ###
                ### Can use for location if disabling GPS Sensor

                #geo = world.get_map().transform_to_geolocation(logger.actorId.get_location())
                #msg = geo.__str__() + '@'
                #logger.setNmea(msg)
                ###########
              
                ### ACCESS/SEND X/Y/Z FROM LAST TICK  #####
                ### Can use for location if disabling GPS Sensor

                #msg = logger.actorId.get_location().__str__() + '@'
                #logger.setNmea(msg)
                ########
                
                # Get the speed of Trike
                getSpeed(logger.actorId, logger)
                
                # Send most current data
                # ORDER MATTERS FOR ARDUINO CODE
                logger.sendCyclometer(ser)
                logger.sendNmea(ser)
                
                ### FOR SENSOR USAGE ###
                logger.setListen()
                #nmeaSensor.listen(lambda data: nmeaGPS.consoleout(data,logger))
                #############
    
    except KeyboardInterrupt:
        print('\ndestroying %d sensors' % len(sensors))
        for x in sensors:
            carla.command.DestroyActor(x)
        print('\ndestroying vehicle')
        actor.destroy()
        return
    
'''
Executes each time CARLA updates sensor data.  Obtains the speed of actor it is attached to.
'''
def getSpeed(actor, logger):
    circumference = .39 ##m
    #actor = next((x for x in world.get_actors() if x.id == vehicles[vIndex][0]), None)
    velocity = actor.get_velocity()
    magnitude = math.sqrt(velocity.x*velocity.x + velocity.y*velocity.y+velocity.z*velocity.z) ##m/s
    if(magnitude < 0.8333): ##random num between .5 and 1.5 when less than 3
        magnitude = random.uniform(.139, .417)
    else: ##add random error scaled by inverse of circumfrence
        magnitude += random.uniform(-.5,.5)*(1/circumference)
    
    #change m/s to s/pulse
    wait = (1 / magnitude) * circumference
    asString = "{:.3f}".format(wait) + '@'  #  The @ char is used to signify the end of a message
    
    logger.setCyclometer(asString)
    #dueNative.write(asString.encode('utf-8'))
    
if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')