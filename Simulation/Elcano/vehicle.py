import serial
import Carla

#Vehicle class, contans
class Vehicle:

    #Constructor for vehicle class
    def __init__(self):

        #Variables related to simulator
        self.sensors = []
        self.speed = 0
        self.steeringangle = 0

        #Variables for vehicle
        #Pulled these from old code, not sure use of them.
        #self.stopListen
        #self.pulse

    #Initialize connection to simulator and spawn vehicle
    def connectToSim(self, host = 'localhost', port = 2000, headless = False):

        #Attempt to connect to the Carla server, timeout after 5 seconds
        self.client = Carla.Client(host, port)
        self.client.set_timeout(5.0)

        #Get the world from the server
        self.world = self.client.get_world()

        #Load Carlas blueprint library
        self.blueprint_library = self.world.get_blueprint_library()

        #Grab all the car blueprints from library
        vehicle = self.blueprint_library.find('vehicle.bmw.isetta')

        #Spawns trike at spawn point
        spawn = Carla.Transform(Carla.Location(x=15, y=20, z=5), Carla.Rotation(yaw=180))
        self.actor = self.world.spawn_actor(vehicle, spawn)

        #Move server viewpoint to trike, will move this out of function with
        #different options in UI for camera setup.
        self.world.get_spectator().set_transform(self.actor.get_transform())

        if(headless):
            #Enable headless mode
            settings = self.world.get_settings()
            settings.no_rendering_mode = True
            world.apply_settings(settings)

        self.spawnSensors()

    #Initialize connection to routerboard
    def connectToRouter(self, port = 'COM4'):

        #Set COM port for router communication
        self.routerboard = serial.Serial(port, baudrate = 115200, timeout=5)

        #Notify Due that system is starting
        self.routerboard.write('f'.encode('utf-8')) 

    #For spawning all sensors onto vehicle, add sensors here.
    def spawnSensors(self):

        #Create and add all desired sensors
        self.sensors.append(NMEA(self.world, self.actor))
        self.sensors.append(IMU(self.world, self.actor))

    #For destroying the vehicle in simulator
    def destroy(self):
        for sensor in sensors:
            sensor.destroy()
        actor.destroy()

    #Demo code, in the works.
    def demo(self):

        while True:
            # Wait for ready queue from due
            if self.routerboard.in_waiting:
                
                ''' 
                ADD SERIAL READS FOR ACTUATION HERE 
                
                Currently just clears an arbitrary char in buffer sent from
                router Due
                '''

                ## Receive in order: throttle, steer, brake
                t = float(self.routerboard.readline().decode('ASCII'))
                s = float(self.routerboard.readline().decode('ASCII'))
                b = float(self.routerboard.readline().decode('ASCII'))
                
                actor.apply_control(Carla.VehicleControl(throttle=t,steer=s,brake=b))

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
                
                
        
    #Copied this function over will likely end up erased
    def getSpeed(self, logger):

        actor = self.actor

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

############ Will move these sensors into their own import ############
#Each sensor will likely be sublasses of greater Sensor class
#Will be able to determine values from carla and send them to router

class NMEA:
    def __init__(self, world, actor):

        # Attach nmeaSensor to trike (speed directly taken from trike actor)
        blueprint = world.get_blueprint_library().find('sensor.other.gnss')
        transform = Carla.Transform(Carla.Location(x=0.8, z=1.7))
        self.sensor = world.spawn_actor(blueprint,transform,actor)
        
class IMU:
    def __init__(self, world, actor):

        #Get sensor and spawn it onto the vehicle
        #Failed to find sensor in test, will look into this
        #blueprint = world.get_blueprint_library().find('sensor.other.imu')
        #transform = Carla.Transform(Carla.Location(x=0.8, z=1.7))
        #self.sensor = world.spawn_actor(blueprint, transform, actor)

        self.name = IMU
