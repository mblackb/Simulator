import math
import Carla

class SimulatedVehicle:
    """
    Vehicle class for simulator, acts as the vehicle object in carla. Contains
    all the sensors we place on vehicle in the simulator and their data. Will
    be used to execute commands and give data on vehicle.
    """

    #Constructor for vehicle class
    def __init__(self):

        #Variables related to simulated vehicle
        self.sensors = []
        self.speed = 0
        self.steeringangle = 0
        self.throttle = 0
        self.steering = 0
        self.braking = 0

    #Initialize connection to simulator and spawn vehicle
    def connectToSim(self, host = 'localhost', port = 2000, headless = False):
        """
        Connects the trike to the simulator server

        Accepted params:
        host - string for server location, default localhost
        port - int for port of server, default 2000
        headless - boolean for a headless server, default False
        """
        
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

        if(headless):
            #Enable headless mode
            settings = self.world.get_settings()
            settings.no_rendering_mode = True
            self.world.apply_settings(settings)

        #spawn the sensors for the vehicle
        self.spawnSensors()

    #For spawning all sensors onto vehicle, add sensors here.
    def spawnSensors(self):

        #Create and add all desired sensors
        self.sensors.append(Camera(self.world, self.actor))
        self.sensors.append(GNSS(self.world, self.actor))
        self.sensors.append(IMU(self.world, self.actor))


    #For destroying the vehicle in simulator
    def destroy(self):
        for sensor in self.sensors:
            sensor.sensor.destroy()
        self.actor.destroy()
        
    def getSpeed(self):
        #Convert velocity into speed
        velocity = self.actor.get_velocity()
        self.speed = math.sqrt(velocity.x*velocity.x + velocity.y*velocity.y+velocity.z*velocity.z)
        return self.speed

    def getSteeringAngle(self):
        #Convert steering -1 to 1 to -90 to 90 degrees
        self.steeringangle = self.steering*90
        return self.steeringangle

    def updateThrottle(self, t):
        self.throttle = t
        self.actor.apply_control(Carla.VehicleControl(throttle=self.throttle,steer=self.steering,brake=self.braking))

    def updateSteering(self, s):
        
        self.steering = s
        self.actor.apply_control(Carla.VehicleControl(throttle=self.throttle,steer=self.steering,brake=self.braking))
    
    def updateBraking(self, b):
        self.braking = b
        self.actor.apply_control(Carla.VehicleControl(throttle=self.throttle,steer=self.steering,brake=self.braking))


        

############ Will move these sensors into their own import ############
#Each sensor will likely be sublasses of greater Sensor class
#Will be able to determine values from carla and send them to router
#
#Need to move sensor function from previous implementation to here.

class GNSS:
    def __init__(self, world, actor):

        # Attach nmeaSensor to trike (speed directly taken from trike actor)
        blueprint = world.get_blueprint_library().find('sensor.other.gnss')
        transform = Carla.Transform(Carla.Location(x=0.8, z=1.7))
        self.sensor = world.spawn_actor(blueprint,transform,attach_to=actor)

        #use built in listen function to help update camera, hacky fix for an issue
        #with pinning camera to vehicle
        #self.sensor.listen(lambda data: self.updateCamera(world))

    def updateData(self):
        self.latitude = self.sensor.latitude
        self.longitude = self.sensor.longitude
        self.altitude = self.sensor.altitude

    #hacky fix for keeping camera locked on car will figure something out better.
    #possibly use pygame for a game hud and display
    def updateCamera(self, world):
        world.get_spectator().set_transform(self.sensor.get_transform())

    
        
class IMU:
    def __init__(self, world, actor):

        #Get sensor and spawn it onto the vehicle
        blueprint = world.get_blueprint_library().find('sensor.other.imu')
        transform = Carla.Transform(Carla.Location(x=0.8, z=1.7))
        self.sensor = world.spawn_actor(blueprint, transform, actor)

        self.name = IMU

    def updateData(self):
        #Do the thing with the data Brandon.
        #You got this, change values to whatever structure you need for it

        self.accelerometer = 0
        self.gyroscope = 0
        self.compass = 0


class Camera:
    def __init__(self, world, actor):

        # Find the blueprint of the sensor.
        blueprint = world.get_blueprint_library().find('sensor.camera.rgb')

        # Modify the attributes of the blueprint to set image resolution and field of view.
        blueprint.set_attribute('image_size_x', '720')
        blueprint.set_attribute('image_size_y', '480')
        blueprint.set_attribute('fov', '110')

        # Set the time in seconds between sensor captures
        blueprint.set_attribute('sensor_tick', '1.0')

        # Provide the position of the sensor relative to the vehicle.
        transform = Carla.Transform(Carla.Location(x=0.8, z=1.7))


        # Tell the world to spawn the sensor, don't forget to attach it to your vehicle actor.
        self.sensor = world.spawn_actor(blueprint, transform, attach_to=actor)


        
    