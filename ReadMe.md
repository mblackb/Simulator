# Documentation for Elcano Simulator Code

**UW Bothell, 2019**

**Advisor :** *Tyler Folsom*

**Team :** *Zach Gale, Jonah Lim, Matthew Moscola, Francisco Navarro-Diaz*

## Purpose:
The following is a generalized list of the desired functionality of the simulator.
 - Simulate data for all sensors associated with Elcano Trike.
 - Simulate Elcano behavior to throttle, brake, and steering.
 - Have low and high-level boards function normally with the simulated sensor data without knowing data is being simulated.
 - Allow low and high-level board interaction to be tested.
 - Allow autonomous driving to be tested.
## Components:
#### Simulation:
Code to be executed by a computer that has access to a Carla server, either locally, or through a network.  Controls all objects in Carla simulation.  Retrieves sensor data from Carla, and sends router board.  Also retrieves actuation instructions from the router board and sends to Carla.
#### Router Board:
Code to be executed on the Arduino Due that functions as the router board.  Routes drive commands from low-level to Carla.  Also routes sensor data from Carla to high/low-level boards.
#### High Level CAN Demo
Simple demo code for high-level board that sends drive CAN messages repeatedly to the CAN Bus.
#### Elcano C2 LowLevel
Current Elcano low-level code with minor adjustments.  Adjustments discussed in detail later in doc.
## Current Functionality of Simulation:
At the end our Autumn 2019 Capstone, we were able to deliver a demo that demonstrated communication between all components; high-level board (Arduino Due), low-level board (Arduino Mega 2560) with shield running low level code, router board (Arduino Due), and the instance of Carla.  The instructions on how to reproduce the demo will be given later in this document. The primary roadblock that prevented further development into this project was the state of the CAN system of the Elcano trike (particularly the transceiver board which seems to have a flawed design).  These are the current features of the simulator.
 - Implementation of USB serial communication between router board and
   Computer running simulator.py.  Both entities (Computer and Arduino
   Due) can send and receive data through USB connection.
- Implementation of a cyclometer.  Router board can receive the current speed of the simulated vehicle in Carla and convert it into an interrupt-based cyclometer pulse, based on the wheel dimension of the Elcano.  Also considers the random error of cyclometer present on the Elcano trike.  This can be debugged with the built-in LED on the router board.
- Implementation of NMEA GPS sensor.  Carla is able the output NMEA GPS data to the router board 10 times per second, effectively simulating the GPS sensor on the high-level board.  Router board is successfully able to output it to UART serial which is how it is transmitted to the high-level board.
- Implementation of timing structure.  Computer running simulator.py waits for data to be interpreted from the router board while the router board executes its update 10 times per second.  This is the desired speed for both sensor data and actuation data to be updated.
- Implementation of vehicle control updaters through throttle, brake, and steering data.  The computer running simulator.py can take desired throttle, brake, and steering and convert them into corresponding commands for Carla using Carla API. These commands move the vehicle within Carla.  This is the primarily what the demo shows.
## Demo Guide (How to Set-Up)
The following is a guide to set up the Elcano simulator.
#### Materials:
- 2 Arduino Due.
- 1 low-level board with Arduino Mega 2560.
- 1 male to female DB15 ribbon cable.
- 2 USB A male to USB Micro B male cables.
- 1 USB A male to B male cable.
- 1 computer capable of running Carla (only tested with Windows 10).
- 1 laptop/computer to interface between Carla and router board (optional, if computer can handle Carla easily you may want to just run this code on the same computer).  Only tested with Windows 10.
- 1 CAN Transceiver board (MCP2550 3.3V preferred, not tested with 5V).  Component used for demo; Waveshare SN65HVD230 CAN Board ([https://www.amazon.com/gp/product/B00KM6XMXO/ref=ppx_yo_dt_b_asin_title_o01_s00?ie=UTF8&psc=1](https://www.amazon.com/gp/product/B00KM6XMXO/ref=ppx_yo_dt_b_asin_title_o01_s00?ie=UTF8&psc=1)).
- 1 breadboard.
- 1 male to female wire, 10+ male to male wires.
#### Setup:
- Firstly get everything connected.  Use the USB cables to connect all your Arduinos to your computer.  Make sure one Arduino (router board) has both programming and native ports connected to the computer if you wish to debug.
- Programming port is used to upload code and communicate with the router board, native port is used to debug to serial com.
- Screw in 3 wires for CAN hi, lo, and ground for the low-level board.  Also connect the DB15 ribbon cable to the low level board.
- On the ribbon cable, attach a wire to the middle pin on the row with less pins.  This connection will route to pin A1 on the router board.  This is the throttle controller.
- On the low-level board, attach a wire to pin 37.  This connection will route to pin 9 on the router board.  This is the brake controller.
- Put your can transceiver in the breadboard and make sure it receives the proper voltage (Due has 3.3 and 5 V ports).
- Using the breadboard, connect the CAN hi, lo, and ground of the low-level board and the CAN transceiver.
- Using the Arduino IDE, find out which COM is associated with the router board.
- Run the ipconfig command through cmd on the windows computer running Carla.
- Replace the ip address and serial COM settings in the simulator.py file with the values just found.
- Make sure you have the following libraries installed for Arduino.
- CAN_BUS_SHIELD by Seeed-Studio
  - [https://github.com/Seeed-Studio/CAN_BUS_Shield](https://github.com/Seeed-Studio/CAN_BUS_Shield)
- PID by Brett Beauregard
  - [https://github.com/br3ttb/Arduino-PID-Library](https://github.com/br3ttb/Arduino-PID-Library)
- Pin Change Interrupt by Nico Hood
  - [https://github.com/NicoHood/PinChangeInterrupt](https://github.com/NicoHood/PinChangeInterrupt)
- MCP48x2 DAC by Jonas Forsberg
  - [https://github.com/SweBarre/MCP48x2](https://github.com/SweBarre/MCP48x2).
- Arduino Due Timer Interrupts by Ivan Seidel
  - [https://github.com/ivanseidel/DueTimer](https://github.com/ivanseidel/DueTimer)
- Previous version of an Arduino Due CAN Bus library from [https://github.com/collin80/due_can](https://github.com/collin80/due_can) is used but the necessary files are included within the repository.
- Upload all codes to their corresponding Arduinos.  The low-level code used for this demo is included in the repository.  The high level board should be uploaded with the High_level_CAN_DEMO code.  Before uploading high-level code, hardcode the CAN message you want being sent.  CAN message structure can be found here; [https://www.elcanoproject.org/wiki/Communication](https://www.elcanoproject.org/wiki/Communication).  The message being sent in the demo are drive instructions with CANID 0x350.
- Observe behavior in simulator.  To change drive instructions in real-time, simply reupload high-level code with a new command and it should be reflected in the simulation.
- If car gets stuck, kill current simulator.py process and restart.
## Out of Scope Functionality Tested/Implemented
- Our project required us to delve into pieces of code within the Elcano project outside of our scope.  Understanding of how sensors are read by the high and low-level boards is necessary for the simulation of the sensors.  We were able to fix some issues regarding the CAN communication between the high and low-level boards.  The modified low-level code was not pushed to the main low-level code folder in the repository on Github, however, it is included within the simulator documentation.  The following were implemented.
- Unification of the proposed CAN message structure and the actual CAN message processing on the low-level code for CAN messages with ID 0x350 (High-level drive instructions).  These are CAN messages the high-level will send to the low level to facilitate autonomy.
- Fixing the DAC output voltage by using the MCP48x2 library by Jonas Forsberg.
## To Do (Software):
#### High Priority:
- Simulate gyro sensor
- Simulate accelerometer/magnetometer sensor.
- Simulate wheel angle sensor.
- Implement steering actuation when low-level CAN processing of steering command is fixed.
- Initialize CAN with high-level board code and incorporate in demo instead of separate CAN transceiver.
#### Medium Priority:
- Simulate ScanseSweep.
- Implement receiver board when board is fully fixed.
- Incorporate Elcano trike physics to simulated vehicle.
#### Low Priority
- Simulate camera sensor.
## Bugs Within Simulator
- Occasionally the simulator and router board will desync and the NMEA GPS messages received by the router board will be faulty.  This is usually followed by a crash of simulator.py.  If not, simply press Ctrl^C and restart simulator.py.
- If simulator.py unexpectedly terminates, the created actors and sensors in Carla will not be properly destroyed.  This usually does not cause problems for the simulation but proper termination in the future would be safer.  For now if it is causing problems simply restart Carla.
- Carla will never stop outputting sensor data even if asked to stop.  This is a bug with Carla and not in our scope.
- Most malfunctions can be temporarily fixed by restarting the simulator.py instance.
## Other Elcano Bugs Found
- While we were able to fix the code for the low-level CAN interpretation of the throttle and brake commands from high-level CAN messages, the PWM steering response was not fully tested.  We were unable to get the low-level board to output the correct PWM for the sent CAN messages and this needs more testing.
- The receiver board cannot get CAN to initialize.  Voltage testing confirmed that the CAN components (MCP2515 and MCP 2550) are receiving the correct operating voltages (5v).  Also, multiple CAN libraries besides the suggested Seeed Studio library were tested with no success (SparkFun CAN library was one of the tested libraries).  Possible problem is incompatibility between the 5v CAN components and the 3.3V SAMD21 board.  Another possible problem is design flaw of pcb.
- Low-level processing of CAN messages for ID 0x350 is faulty.  Bit shifting and produced output do not match the CAN guidelines on the wiki.  This was fixed in the low-level code included in the simulator folder but not pushed to the Elcano Github repository.
## To Note:
- Currently, the order that you output data from the router board to the computer running simulator.py is non-trivial.  Data is interpreted based on the order which it is received.
- Carla sensor listening was flawed at the time this was made; .stop() would not stop the sensor from executing its attached function at each tick of Carla.  Therefore speed was implemented without the use of sensors, and instead uses a command that accesses the speed only when needed.  This is the suggested method until sensors are fixed.  Certain sensors such as NMEA GPS, however, are only implementable using Carla sensors.  Additional information can be found on Carla Documentation.
- In PowerShell, in CarlaUE4 path, run Carla as: ***Start-Process CarlaUE4 -ArgumentList “—quality-level=Low”*** to lower gpu load.
- Headless mode can be enabled for complete removal of graphics rendering, however, visual debugging is extremely useful.
