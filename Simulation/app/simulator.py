
'''

Elcano Carla Simulator Capstone
UW Bothell, 2020

Advisor : Tyler Folsom

Team 2019 : Zach Gale, Jonah Lim, Matthew Moscola, Francisco Navarro-Diaz
Team 2020 : Colton Sellers, Brandon Thompson

simulator.py

Version: 1.0

The main purposes of this program are:
    - Control creation of actors within Carla.
    - Retrieve simulated sensor data from actors and send to router board.
    - Interpret actuation data from router board and send actuation commands to Carla.

Much of the code that prepares the simulation (spawning actors and sensors) can be found in the examples
that come with the CARLA simulator download.

CARLA open-source simulator can be found here: http://Carla.org/

'''

#External imports
import sys
import logging
import pygame
import datetime
import math
import os


#import Carla and and Sensors
import Carla
import Elcano


def main(COMPort = 'COM10', host = 'localhost', port = 2000):
    """
    Take in settings from form or command line, start logging, build client object
    enter control loop

    Accepted Params:
    COMPort
    host
    port
    """

    #Set logging level
    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)
    logging.info('listening to server %s:%s', host, port)


    #Start pygame and build window and hud
    pygame.init()
    pygame.font.init()

    try:
        display = pygame.display.set_mode(
        (1280, 720),
        pygame.HWSURFACE | pygame.DOUBLEBUF)

        clock = pygame.time.Clock()
             
        hud = HUD(1280,720)
        world = Client(hud, host, port)
       
        #MOVE THIS INTO CLIENT CLASS!!!!!!!!!!!
        #Create the simulated vehicle and connect to server
        trike = Elcano.SimulatedVehicle(world.worldObj, hud)
        world.setVehicle(trike)

        #Define the controller of the vehicle
        controller = Elcano.RouterboardInterface(COMPort, trike)

        #Enter the main loop
        while True:
            clock.tick_busy_loop(60)
            controller.controlLoop()
            world.tick(clock)
            world.render(display)
            pygame.display.flip()


    finally: 
        #DELETE WORLD/ VEHICLE AND ALL SENSORS!!!!!
        if world is not None:
            trike.destroy()


        pygame.quit()


# ==============================================================================
# -- Client ---------------------------------------------------------------------
# ==============================================================================


class Client(object):
#TODO::
#ADD VEHICLE CLASS UNDER THIS 
#CLEAN AND IMPROVE!!!!

    """
    Meant to represent the connection to the simulated world, all objects that exist in it,
    as well as our view into that world.

    Client owns the display hud and the vehicle

    Maybe move camera manager out of vehicle as it isn't used as a sensor rather than just the
    client window.....
    """

    def __init__(self, hud, host, port):

        #Attempt to connect to the Carla server, timeout after 5 seconds
        self.client = Carla.Client(host, port)
        self.client.set_timeout(5.0)

        #take in HUD
        self.hud = hud

        #Get the world from the server
        self.worldObj = self.client.get_world()
        self.worldObj.on_tick(hud.on_world_tick)
        
        self.recording_enabled = False
        self.recording_start = 0

    def setVehicle(self, vehicle):
        self.vehicle = vehicle

    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self.player.get_world().set_weather(preset[0])


    def tick(self, clock):
        self.hud.tick(self.worldObj, self.vehicle, clock)

    def render(self, display):
        self.vehicle.camera_manager.render(display)
        self.hud.render(display)



# ==============================================================================
# -- HUD -----------------------------------------------------------------------
# ==============================================================================


class HUD(object):
    def __init__(self, width, height):
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        font_name = 'courier' if os.name == 'nt' else 'mono'
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 12 if os.name == 'nt' else 14)
        self._notifications = FadingText(font, (width, 40), (0, height - 40))
        self.help = HelpText(pygame.font.Font(mono, 16), width, height)
        self.server_fps = 0
        self.frame = 0
        self.simulation_time = 0
        self._show_info = True
        self._info_text = []
        self._server_clock = pygame.time.Clock()

    def on_world_tick(self, timestamp):
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame = timestamp.frame
        self.simulation_time = timestamp.elapsed_seconds

    def tick(self, world, vehicle, clock):
        self._notifications.tick(world, clock)
        if not self._show_info:
            return
        t = vehicle.actor.get_transform()
        v = vehicle.actor.get_velocity()
        c = vehicle.actor.get_control()

        #Handling compass from IMU
        compass = vehicle.IMUSensor.compass
        heading = 'N' if compass > 270.5 or compass < 89.5 else ''
        heading += 'S' if 90.5 < compass < 269.5 else ''
        heading += 'E' if 0.5 < compass < 179.5 else ''
        heading += 'W' if 180.5 < compass < 359.5 else ''

        self._info_text = [
            'Server:  % 16.0f FPS' % self.server_fps,
            'Client:  % 16.0f FPS' % clock.get_fps(),
            '',
            'Vehicle: % 20s' % "Elcano",
            'Simulation time: % 12s' % datetime.timedelta(seconds=int(self.simulation_time)),
            '',
            'Speed:   % 15.0f km/h' % (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)),
            u'Compass:% 17.0f\N{DEGREE SIGN} % 2s' % (compass, heading),
            'Accelero: (%5.1f,%5.1f,%5.1f)' % (vehicle.IMUSensor.accelerometer),
            'Gyroscop: (%5.1f,%5.1f,%5.1f)' % (vehicle.IMUSensor.gyroscope),
            'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (t.location.x, t.location.y)),
            'GNSS:% 24s' % ('(% 2.6f, % 3.6f)' % (vehicle.GNSSSensor.latitude, vehicle.GNSSSensor.longitude)),
            'Height:  % 18.0f m' % t.location.z,
            '']

        if isinstance(c, Carla.VehicleControl):
            self._info_text += [
                ('Throttle:', c.throttle, 0.0, 1.0),
                ('Steer:', c.steer, -1.0, 1.0),
                ('Brake:', c.brake, 0.0, 1.0),
                ('Reverse:', c.reverse),
                ('Hand brake:', c.hand_brake),
                ('Manual:', c.manual_gear_shift),
                'Gear:        %s' % {-1: 'R', 0: 'N'}.get(c.gear, c.gear)]

        elif isinstance(c, Carla.WalkerControl):
            self._info_text += [
                ('Speed:', c.speed, 0.0, 5.556),
                ('Jump:', c.jump)]


    def toggle_info(self):
        self._show_info = not self._show_info

    def notification(self, text, seconds=2.0):
        self._notifications.set_text(text, seconds=seconds)

    def error(self, text):
        self._notifications.set_text('Error: %s' % text, (255, 0, 0))

    def render(self, display):
        if self._show_info:
            info_surface = pygame.Surface((220, self.dim[1]))
            info_surface.set_alpha(100)
            display.blit(info_surface, (0, 0))
            v_offset = 4
            bar_h_offset = 100
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x + 8, v_offset + 8 + (1.0 - y) * 30) for x, y in enumerate(item)]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                        f = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect((bar_h_offset + f * (bar_width - 6), v_offset + 8), (6, 6))
                        else:
                            rect = pygame.Rect((bar_h_offset, v_offset + 8), (f * bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item:  # At this point has to be a str.
                    surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(surface, (8, v_offset))
                v_offset += 18
        self._notifications.render(display)
        self.help.render(display)


# ==============================================================================
# -- FadingText ----------------------------------------------------------------
# ==============================================================================


class FadingText(object):
    def __init__(self, font, dim, pos):
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def tick(self, _, clock):
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        display.blit(self.surface, self.pos)


# ==============================================================================
# -- HelpText ------------------------------------------------------------------
# ==============================================================================


class HelpText(object):
    """Helper class to handle text output using pygame"""
    def __init__(self, font, width, height):
        lines = __doc__.split('\n')
        self.font = font
        self.line_space = 18
        self.dim = (780, len(lines) * self.line_space + 12)
        self.pos = (0.5 * width - 0.5 * self.dim[0], 0.5 * height - 0.5 * self.dim[1])
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)
        self.surface.fill((0, 0, 0, 0))
        for n, line in enumerate(lines):
            text_texture = self.font.render(line, True, (255, 255, 255))
            self.surface.blit(text_texture, (22, n * self.line_space))
            self._render = False
        self.surface.set_alpha(220)

    def toggle(self):
        self._render = not self._render

    def render(self, display):
        if self._render:
            display.blit(self.surface, self.pos)
