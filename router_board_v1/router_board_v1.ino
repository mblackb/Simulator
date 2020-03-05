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

#include "router.h"

#define __arm__

#define PULSE_PIN 5
#define END_MARKER '@'  // char that signifies the end of a block of sensor data
#define BRAKE_PIN 9
#define STEER_PIN 6
#define THROTTLE_PIN A1


//SENSORS TO ADD: wheel angle sensor


// Download from github.com/ivanseidel/DueTimer
//  -Follow instructions on github page to put in Arduino library
#include "DueTimer.h"


// Ensure cyclometer clicked at least once before changing its frequency
bool clicked = false;


// Used for receiving serial ascii messages
const byte numChars = 64;
char receivedChars[numChars];
boolean newData = false;
byte len = 0;
int receivedData = 0;

int loopStart = 0;

//Steering variable setup
volatile int pulseTime = 0;
volatile long streerTimerStart = 0;
bool inSteering = false;
//int steeringRead = 0;
//int steerTime = 0;

// These variables hold actuation data
byte throttle = 0;
byte brake = 0;
byte steering = 0;

byte throttleSent = 0;
byte steeringPrevious = 0;
bool inBraking = false;
bool brakeChange = false;

//Set pins, prepare serial.

void setup() {
  Serial.begin(BAUDRATE);
  SerialUSB.begin(BAUDRATE);

  // Built in built in LED for debug
  pinMode(LED_BUILTIN, OUTPUT);

  // Set pins
  pinMode(BRAKE_PIN, INPUT);
  pinMode(STEER_PIN, INPUT);

  // Start cyclometer pulse with arbitrarily large value
  Timer3.attachInterrupt(sendPulse).start(2000000);

  // Initialize with no steering, brakes, or throttle
  setActuation(throttle, steering, brake);

  // Wait until start msg from laptop
  while (SerialUSB.available() <= 0) {}
  SerialUSB.read(); // flush char in SerialUSB
  delay(100);   // Allow time for some data from Carla to be captured
  loopStart = millis();

  // Steering and braking handled in
  attachInterrupt(STEER_PIN, manageSteering, CHANGE);
  attachInterrupt(BRAKE_PIN, manageBrake, CHANGE);
}


/*
  Each loop does 3 things
  - Reads actuation signals from low level board, updates last seen values.
  - Sends formatted actuation data to PC connected to Carla.
  - Retrieves simulated sensor data from the PC.


  TODO:
  -MOVE SENSOR READS TO INTERUPT
  -USE LOOP TO WRITE DATA TO CARLA
  -USE LOOP TO READ DATA TO CARLA
  -MOVE CONVERSIONS TO PYTHON CODE
*/
void loop() {
  //int start = millis();
  receivedData = 0;

  /////////////////////////////////////////////////
  //  Update actuation instructions from low level
  //  Throttle : from 50 to 225 (After reading signal about 40 to 140), analogRead from pin A1
  //  Brake: on or off, digital pin 9
  //  Steering: Pulse width, digital pin 6
  //////////////////////////////////////////////////

  //We don't need to adjust the throttle if we are braking.
  if (!inBraking) {
    // Read throttle and send to Carla if it has changed
    int throttleRead = analogRead(THROTTLE_PIN);
    throttle = throttleRead >> 2;  // Read with 8 bit resolution

    //only send throttle data to Carla if it is different.
    if (throttle > (throttleSent + 10) || throttle < (throttleSent - 10)) {
      throttleSent = throttle;
      sendActuation(throttleCommand, &throttleSent, THROTTLEBYTES);
      Serial.print("Throttle: ");
      Serial.println(throttleSent);
    }
  }

  // If brake state has changed, send new brake value to Carla
  if (brakeChange) {
    sendActuation(brakeCommand, &brake, BRAKEBYTES);
    brakeChange = false;
    Serial.print("Brake: ");
    Serial.println(brake);
  }

  // send steering data if it has been updated
  if (inSteering) {
    sendActuation(steerCommand, &steering, STEERBYTES);
    inSteering = false;
    Serial.print("Steering: ");
    Serial.println(steering);
  }







  ///////////////////////////////////////
  // Send actuation control data to Carla
  ///////////////////////////////////////

  //  sendActuation();

  ////////////////////////////////////////////
  // Receive sensor data from Carla
  // Process data based on order received
  // Current order:
  //  1st) Wait time in between cyclometer pulse in seconds
  //  2nd) GPS data
  /////////////////////////////////////////////
  //**********************To DO****************************
  //***************Update recieve packets from Carla*******
  // Get time in between cyclometer pulse
  // recvWithEndMarker();
  // receiveNewData('s');

  // Get GPS data
  // recvWithEndMarker();
  // receiveNewData('g');

  // Gotta send it back

  /////////////////////////////////////////////////////
  // Finish data transactions
  /////////////////////////////////////////////////////

  // Make loop only happen once every 1/10th of a second
  //while (millis() - start <= 100) {}

}


//Receives data from the programming port.
void recvWithEndMarker() {
  static byte ndx = 0;
  char rc;

  while (SerialUSB.available() > 0 && newData == false) {
    rc = SerialUSB.read();

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
void setActuation(byte t, byte s, byte b) {
  throttle = t;
  brake = b;
  steering = s;
}

// Sends data to carla. Command list needs match command list for
void sendActuation(carlaCommand mode, byte data[], int byteCount) {
  //SerialUSB.write(mode);
  char sendData[byteCount];
  for (byte i = 0; i < byteCount; i++) {
    sendData[i] = data[i];
  }

  SerialUSB.write(mode);
  SerialUSB.write(sendData, byteCount);
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

// Expected PWM frequency is 1KHz, so should be 1000 microseconds max
// Steering ISR, is called by interrupt by steering signal
void manageSteering() {
  //noInterrupts();
  //if the pin has gone HIGH, record the microseconds since the Arduino started up
  if (digitalRead(STEER_PIN) == HIGH)
  {
    streerTimerStart = micros();
  }
  //otherwise, the pin has gone LOW
  else
  {
    //only worry about this if the timer has actually started
    if (streerTimerStart != 0)
    {
      //record the pulse time
      steeringPrevious = steering;
      pulseTime = ((volatile long)micros() - streerTimerStart);
      steering = map(pulseTime, 0, 2040, -128, 127);
      if (steering != steeringPrevious)
        inSteering = true;
      //restart the timer
      streerTimerStart = 0;
    }
  }



  //steeringRead = pulseIn(STEER_PIN, HIGH);
  //steeringRead = map(steeringRead, 1000, 1850, 0,18);

  //if (steeringRead < 0) {
  //  if (steeringRead < -9) steeringRead = -9;
  //}

  //steering = posdeci + steeringRead + nl;
  //interrupts();
}


// ISR for brake pin
void manageBrake () {
  if (digitalRead(BRAKE_PIN) == HIGH) {
    brake = 0xFF;  // Feel free to change
    throttle = 0;
    inBraking = true;
  }
  else {
    brake = 0;
    inBraking = false;
  }
  brakeChange = true;
  //Serial.print(brake);
}
