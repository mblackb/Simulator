import serial
import Carla




class Vehicle:
    def __init__(self):

        #Carla vehicle options
        vehiclesOpt = ["trike"]
        vehicle = None  # actor ID of the spawned vehicle
        sensorsOptDisp = ["nmeaGPS","rotSensor","numGPS"]
        sensorsOpt = ["nmeagps","rotsensor","numgps"]
        sensors = []





    def initialize(self, host = 'localhost', port = 2000):

        #Set COM port for router communication
        #self.ser = serial.Serial('COM4',baudrate = 115200,timeout=1)

        #Attempt to connect to the Carla server, timeout after 5 seconds
        self.client = Carla.Client(host, port)
        self.client.set_timeout(5.0)

        #Get the world from the server
        self.world = self.client.get_world()

        #Load Carlas blueprint library
        blueprint_library = self.world.get_blueprint_library()

        #Grab all the car blueprints from library
        vehicle = blueprint_library.find('vehicle.bmw.isetta')

        #Spawns trike at spawn point
        spawn = Carla.Transform(Carla.Location(x=15, y=20, z=5), Carla.Rotation(yaw=180))
        self.actor = self.world.spawn_actor(vehicle, spawn)

        ### SET SIM VIEWPOINT TO VEHICLE ####
        self.world.get_spectator().set_transform(self.actor.get_transform())

    

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