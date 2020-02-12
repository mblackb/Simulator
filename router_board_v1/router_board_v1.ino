/*
Elcano Carla Simulator Capstone
UW Bothell, 2019
Advisor : Tyler Folsom
Team : Team: Zach Gale, Jonah Lim, Matthew Moscola, Francisco Navarro-Diaz
router_board_v1.ino
Version: 1.0
The main purposes of this program are:
    - Route actuation controls from low-level to Carla
    - Route simulated sensor data from Carla to low/high Level
** Current implementation relies on order of sending to identify data
*/

#define __arm__

#define PULSE_PIN 5
#define END_MARKER '@'  // char that signifies the end of a block of sensor data
#define BRAKE_PIN 6
#define STEER_PIN 9
#define THROTTLE_PIN A1

//SENSORS TO ADD: wheel angle sensor


// Download from github.com/ivanseidel/DueTimer
//  -Follow instructions on github page to put in Arduino library
#include "DueTimer.h"


// Ensure cyclometer clicked at least once before changing its frequency
boolean clicked = false;

// Used for receiving serial ascii messages
const byte numChars = 64;
char receivedChars[numChars];
boolean newData = false;
byte len = 0;
int receivedData = 0;

int loopStart = 0;

//Steering variable setup
bool inSteering = false;
int steeringRead = 0;
int steerTime = 0;

// For formatting data to be sent to Carla
String nl = "\n";
String posdeci = "0.";
String negdeci = "-0.";

// These variables hold actuation data
String throttle = "0.0\n";
String brake = "0.0\n";
String steering = "0.0\n";


//Set pins, prepare serial.

void setup() {
  Serial.begin(115200);
  SerialUSB.begin(115200);

  // Built in built in LED for debug
  pinMode(LED_BUILTIN, OUTPUT);

  // Set pins
  pinMode(BRAKE_PIN, INPUT);
  pinMode(STEER_PIN, INPUT);

  //Serial1.begin(9600);

  // Start cyclometer pulse with arbitrarily large value
  Timer3.attachInterrupt(sendPulse).start(2000000);

  // Initialize with no steering, brakes, or throttle
  setActuation(throttle, steering, brake);

  // Wait until start msg from laptop
  while (Serial.available() <= 0) {}
  Serial.read(); // flush char in serial
  delay(100);   // Allow time for some data from Carla to be captured
  loopStart = millis();

  attachInterrupt(STEER_PIN, manageSteering, CHANGE);
}


/*
Each loop does 3 things
  - Reads actuation signals from low level board, updates last seen values.
  - Sends formatted actuation data to PC connected to Carla.
  - Retrieves simulated sensor data from the PC.
*/
void loop() {
  int start = millis();
  receivedData = 0;

  /////////////////////////////////////////////////
  // Update actuation instructions from low level
  //  Throttle : from 50 to 225 (After reading signal about 40 to 140), analogRead from pin A1
  //  Brake: on or off, digital pin 9
  //  Steering: Pulse width, digital pin 6
  //////////////////////////////////////////////////

  // Read throttle, convert.  Carla throttle values from 0.0 to 1.0
  //  Set throttle global var to a float from 0.0 1.0 as a String ending with new line
  int throttleRead = analogRead(THROTTLE_PIN);
  throttleRead = throttleRead >> 2;  // Read with 8 bit resolution
  // Temporary: map values to 0.1 to 0.9

  if (throttleRead >= 20) {
    throttleRead = map(throttleRead, 30, 140, 1, 9);

    if (throttleRead > 9){
        throttleRead = 9;
    }

    throttle = posdeci + throttleRead + nl;
    //SerialUSB.println(throttleRead);
  } else throttle = "0.0\n";

    //**CAN TURN THIS INTO A REAL VALUE RANGE FOR CARLA FROM ELCANO**
  // Read brake, turn brake on or off.  Carla brake values from 0.0 to 1.0
  //  Set brake global var to a float from 0.0 1.0 as a String ending with new line
  if (digitalRead(BRAKE_PIN) == HIGH) {
    brake = "0.3\n";  // Feel free to change
    throttle = "0.0\n";
  }
  else brake = "0.0\n";
  //Serial.print(brake);





  ///////////////////////////////////////
  // Send actuation control data to Carla
  ///////////////////////////////////////

  sendActuation();

  ////////////////////////////////////////////
  // Receive sensor data from Carla
  // Process data based on order received
  // Current order:
  //  1st) Wait time in between cyclometer pulse in seconds
  //  2nd) GPS data
  /////////////////////////////////////////////

  // Get time in between cyclometer pulse
  recvWithEndMarker();
  receiveNewData('s');

  // Get GPS data
  recvWithEndMarker();
  receiveNewData('g');

  //Gotta send it back

  /////////////////////////////////////////////////////
  // Finish data transactions
  /////////////////////////////////////////////////////

  // Make loop only happen once every 1/10th of a second
  while (millis() - start <= 100) {}

}


//Receives data from the programming port.
void recvWithEndMarker() {
  static byte ndx = 0;
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (rc != END_MARKER) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }

      len = ndx;
    }
    else {
      receivedChars[ndx] = '\0';  //Terminate string
      ndx = 0;
      newData = true;
    }
  }
}

//Does the appropriate action for each set of data retrieved
void receiveNewData(char dataType) {
  if (newData == true) {
    // Identify type of data, process/route data appropriately

    // Data received is time (seconds) in between cyclometer pulse
    if (dataType == 's') {
      double waitTime = atof(receivedChars) * 1000000;  // in microseconds
      //SerialUSB.println(waitTime);
      if (waitTime > 0 && clicked) {
        Timer3.attachInterrupt(sendPulse).start(waitTime);
        clicked = false;
      }
    }

    // Data received is nmea GPS location of trike
    if (dataType == 'g') {
      SerialUSB.println(receivedChars);
      //Serial1.write(receivedChars, len);
    }

    // Clear receive buffer for new data
    newData = false;
    len = 0;
    receivedData++;
  }
}


//Allows you to set throttle, brake and steering in one call.
void setActuation(String t, String s, String b) {
  throttle = t;
  brake = b;
  steering = s;
}


//Sends the last updated values for throttle, steering, and brakes.
void sendActuation() {
  Serial.print(throttle);
  Serial.print(steering);
  Serial.print(brake);
}


/*Sends pulse to low level for speed calculation.  Currently on random digital pin.  Has to be
implemented for corresponding anaolog pin.*/

void sendPulse() {
  //noInterrupts();
  digitalWrite(PULSE_PIN, HIGH);
  digitalWrite(PULSE_PIN, LOW);

  clicked = true;

  // Debug
  Blink();
  //interrupts();
}


//Blink the on-board LED to test the cyclometer
void Blink() {
  digitalWrite(LED_BUILTIN, digitalRead(LED_BUILTIN) ^ 1);
}

//0 - 1024 Digital Value 10bit **WE THINK**
//Steering ISR, is called by interrupt by steering signal
void manageSteering() {
  steeringRead = pulseIn(STEER_PIN, HIGH);
  steeringRead = map(steeringRead, 1000, 1850, 0,18);

  if (steeringRead < 0) {
    if (steeringRead < -9) steeringRead = -9;  
  }

  steering = posdeci + steeringRead + nl;

}