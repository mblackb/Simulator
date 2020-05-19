import math
import Carla
from Carla import ColorConverter as cc
import weakref
import pygame
import numpy as np


class SimulatedVehicle:
    """
    Vehicle class for simulator, acts as the vehicle object in carla. Contains
    all the sensors we place on vehicle in the simulator and their data. Will
    be used to execute commands and give data on vehicle.
    """

    #Constructor for vehicle class
    def __init__(self, world, hud):

        self.world = world

        #Variables related to simulated vehicle
        self.sensors = []
        self.speed = 0
        self.steeringangle = 0
        self.throttle = 0
        self.steering = 0
        self.braking = 0
        self.hud = hud


        #Load Carlas blueprint library
        self.blueprint_library = self.world.get_blueprint_library()

        #Grab all the car blueprints from library
        vehicle = self.blueprint_library.find('vehicle.bmw.isetta')

        #Spawns trike at spawn point
        spawn = Carla.Transform(Carla.Location(x=34, y=7, z=1), Carla.Rotation(yaw=0))
        self.actor = self.world.spawn_actor(vehicle, spawn)

        #spawn the sensors for the vehicle
        self.spawnSensors()

    #For spawning all sensors onto vehicle, add sensors here.
    def spawnSensors(self):

        # Set up the sensors.
        self.GNSSSensor = GNSSSensor(self.world, self.actor)
        self.IMUSensor = IMUSensor(self.world, self.actor)
        self.camera_manager = CameraManager(self.actor, self.world, self.hud, 2.2)
        self.camera_manager.transform_index = 0
        self.camera_manager.set_sensor(0, notify=False)



    #For destroying the vehicle in simulator
    def destroy(self):
    
        self.GNSSSensor.sensor.destroy()
        self.IMUSensor.sensor.destroy()
        self.camera_manager.sensor.destroy()

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


################################# SENSORS  #####################################

# ==============================================================================
# -- GNSSSensor ----------------------------------------------------------------
# ==============================================================================

class GNSSSensor:
    """
    Sensor class for collecting GNSS data from Carla
    """

    def __init__(self, world, actor):


        # Init default values
        self.latitude = 0
        self.longitude = 0

        # Attach nmeaSensor to trike (speed directly taken from trike actor)
        blueprint = world.get_blueprint_library().find('sensor.other.gnss')
        transform = Carla.Transform(Carla.Location(x=1.0, z=2.8))
        self.sensor = world.spawn_actor(blueprint,transform,attach_to=actor)

        #To avoid circular ref we are going to use a weak reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: GNSSSensor._on_gnss_event(weak_self, event))

    #Storing the gps info on every update
    @staticmethod
    def _on_gnss_event(weak_self, event):
        """
        To store the GNSS data every time it changes in CARLA
        """

        self = weak_self()
        if not self:
            return

        self.latitude = event.latitude
        self.longitude = event.longitude
        self.altitude = event.altitude
    
        
# ==============================================================================
# -- IMUSensor -----------------------------------------------------------------
# ==============================================================================

class IMUSensor(object):
    """
    Sensor class for collecting IMU data from Carla
    """

    def __init__(self, world, actor):

        #Default values
        self.accelerometer = (0.0, 0.0, 0.0)
        self.gyroscope = (0.0, 0.0, 0.0)
        self.compass = 0.0

        # Attach the sensor to the vehicle
        bp = world.get_blueprint_library().find('sensor.other.imu')
        self.sensor = world.spawn_actor(bp, Carla.Transform(), attach_to=actor)


        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda sensor_data: IMUSensor._IMU_callback(weak_self, sensor_data))

    @staticmethod
    def _IMU_callback(weak_self, sensor_data):
        """
        To store the IMU data every time it changes in CARLA
        """

        self = weak_self()
        if not self:
            return

        limits = (-99.9, 99.9)
        self.accelerometer = (
            max(limits[0], min(limits[1], sensor_data.accelerometer.x)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.y)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.z)))
        self.gyroscope = (
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.x))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.y))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.z))))
        self.compass = math.degrees(sensor_data.compass)


# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================


class CameraManager(object):
    """
    Sensor class for collecting camera data from Carla

    Completely taken from manualcontrol.py so it is not entirely understood by me.
    """

    def __init__(self, actor, world, hud, gamma_correction):
        self.sensor = None
        self.surface = None
        self.actor = actor
        self.hud = hud
        self.recording = False

        bound_y = 0.5 + self.actor.bounding_box.extent.y
        Attachment = Carla.AttachmentType

        self._camera_transforms = [
            (Carla.Transform(Carla.Location(x=-5.5, z=2.5), Carla.Rotation(pitch=8.0)), Attachment.SpringArm),
            (Carla.Transform(Carla.Location(x=1.6, z=1.7)), Attachment.Rigid),
            (Carla.Transform(Carla.Location(x=5.5, y=1.5, z=1.5)), Attachment.SpringArm),
            (Carla.Transform(Carla.Location(x=-8.0, z=6.0), Carla.Rotation(pitch=6.0)), Attachment.SpringArm),
            (Carla.Transform(Carla.Location(x=-1, y=-bound_y, z=0.5)), Attachment.Rigid)]
        self.transform_index = 1
        self.sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB', {}],
            ['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)', {}],
            ['sensor.camera.depth', cc.Depth, 'Camera Depth (Gray Scale)', {}],
            ['sensor.camera.depth', cc.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)', {}],
            ['sensor.camera.semantic_segmentation', cc.Raw, 'Camera Semantic Segmentation (Raw)', {}],
            ['sensor.camera.semantic_segmentation', cc.CityScapesPalette,
                'Camera Semantic Segmentation (CityScapes Palette)', {}],
            ['sensor.lidar.ray_cast', None, 'Lidar (Ray-Cast)', {}],
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB Distorted',
                {'lens_circle_multiplier': '3.0',
                'lens_circle_falloff': '3.0',
                'chromatic_aberration_intensity': '0.5',
                'chromatic_aberration_offset': '0'}]]
        bp_library = world.get_blueprint_library()

        for item in self.sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(hud.dim[0]))
                bp.set_attribute('image_size_y', str(hud.dim[1]))
                if bp.has_attribute('gamma'):
                    bp.set_attribute('gamma', str(gamma_correction))
                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)
            elif item[0].startswith('sensor.lidar'):
                bp.set_attribute('range', '50')
            item.append(bp)

        self.index = None

    def toggle_camera(self):
        self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
        self.set_sensor(self.index, notify=False, force_respawn=True)

    def set_sensor(self, index, notify=True, force_respawn=False):
        index = index % len(self.sensors)
        needs_respawn = True if self.index is None else \
            (force_respawn or (self.sensors[index][2] != self.sensors[self.index][2]))
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self.surface = None
            self.sensor = self.actor.get_world().spawn_actor(
                self.sensors[index][-1],
                self._camera_transforms[self.transform_index][0],
                attach_to=self.actor,
                attachment_type=self._camera_transforms[self.transform_index][1])

            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        if notify:
            self.hud.notification(self.sensors[index][2])
        self.index = index

    def next_sensor(self):
        self.set_sensor(self.index + 1)

    def toggle_recording(self):
        self.recording = not self.recording
        self.hud.notification('Recording %s' % ('On' if self.recording else 'Off'))

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        if self.sensors[self.index][0].startswith('sensor.lidar'):
            points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 3), 3))
            lidar_data = np.array(points[:, :2])
            lidar_data *= min(self.hud.dim) / 100.0
            lidar_data += (0.5 * self.hud.dim[0], 0.5 * self.hud.dim[1])
            lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (self.hud.dim[0], self.hud.dim[1], 3)
            lidar_img = np.zeros((lidar_img_size), dtype = int)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            self.surface = pygame.surfarray.make_surface(lidar_img)
        else:
            image.convert(self.sensors[self.index][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if self.recording:
            image.save_to_disk('_out/%08d' % image.frame)

