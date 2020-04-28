/*
  Elcano Carla Simulator Capstone
  UW Bothell, 2020
  Advisor : Tyler Folsom
  Team : Team: Launnie Ginn, Willeta Song, Colton Sellers, Brandon Thompson, Mariah Files
  router_board.ino
  Version: 2.0
  The main purposes of this program are:
    - Route actuation controls from low-level to Carla
    - Route simulated sensor data from Carla to low/high Level
** Data being sent to Carla is in the format:
  {Header Byte}{Data Bytes}
  The number of data bytes is determined by the type of data being sent, which is defined by the header byte.
*/

#include "router.h"
#include <Wire.h>

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


//Steering variables
volatile int pulseTime = 0;
volatile long streerTimerStart = 0;
byte steeringPrevious = 0;
bool inSteering = false;
byte steering = 0;


// Throttle Variables
volatile byte throttle = 0;
byte throttleSent = 0;

// Braking Variables
byte brake = 0;
bool inBraking = false;
bool brakeChange = false;

byte accelDataBuffer[ACCELBYTES];
byte magDataBuffer[MAGBYTES];



//Set pins, prepare serial.
void setup() {

  // Serial port used for debug
  Serial.begin(BAUDRATE);
  // Serial USB port used for Carla connection
  SerialUSB.begin(BAUDRATE);

  // Built in built in LED for debug
  pinMode(LED_BUILTIN, OUTPUT);

  // Set pins
  pinMode(BRAKE_PIN, INPUT);
  pinMode(STEER_PIN, INPUT);

  // Start cyclometer pulse with arbitrarily large value
  Timer3.attachInterrupt(sendPulse).start(2000000);

  // Setup I2C for accelerometer and Magnetometer
  // Currently using SDA/SCL for accelerometer and SDA1/SCL1
  // For Magnetometer. Need to work out how to add
  // Gyroscope address as a valid slave address.
  Wire.begin(ACCELADDRESS);
  Wire.onRequest(accelEvent);
  Wire1.begin(MAGADDRESS);       // Accelerometer and magnetometer addresses
  Wire1.onRequest(magEvent);  // register callback function

  // Wait until start msg from laptop
  while (SerialUSB.available() <= 0) {}
  SerialUSB.read(); // flush char in SerialUSB
  delay(100);   // Allow time for some data from Carla to be captured

  // Steering and braking handled in interrupts
  attachInterrupt(STEER_PIN, manageSteering, CHANGE);
  attachInterrupt(BRAKE_PIN, manageBrake, CHANGE);
}




/*
  TODO:
  -MOVE SENSOR READS TO INTERUPT
  -USE LOOP TO WRITE DATA TO CARLA
  -USE LOOP TO READ DATA TO CARLA

*/
void loop() {
  receivedData = 0;

  //We don't need to adjust the throttle if we are braking.
  if (!inBraking) {
    // Read throttle and send to Carla if it has changed
    int throttleRead = analogRead(THROTTLE_PIN);
    throttle = throttleRead >> 2;  // Read with 8 bit resolution

    //only send throttle data to Carla if it is different.
    if (throttle > (throttleSent + THROTTLENOISEMASK) || throttle < (throttleSent - THROTTLENOISEMASK)) {
      throttleSent = throttle;
      sendToCarla(throttleCommand, &throttleSent, THROTTLEBYTES);
      Serial.print("Throttle: ");
      Serial.println(throttleSent);
    }
  }

  // If brake state has changed, send new brake and throttle values to Carla
  if (brakeChange) {
    sendToCarla(throttleCommand, &throttleSent, THROTTLEBYTES);
    sendToCarla(brakeCommand, &brake, BRAKEBYTES);
    brakeChange = false;
    Serial.print("Brake: ");
    Serial.println(brake);
  }

  // send steering data if it has been updated
  if (inSteering) {
    sendToCarla(steerCommand, &steering, STEERBYTES);
    inSteering = false;
    Serial.print("Steering: ");
    Serial.println(steering);
  }

  if (SerialUSB.available()) {
    volatile byte *receiveData = receiveFromCarla(1);
    switch (receiveData[0]) {
      case accelCommand:
        // Get number of bytes associated with Accelerometer data
        receiveData = receiveFromCarla(ACCELBYTES);
        // Put received data into the accelerometer buffer
        loadAccelBuffer(receiveData);
        break;
      case gyroCommand:
        // Get number of bytes associated with gyroscope data
        receiveData = receiveFromCarla(GYROBYTES);
        // TODO: ADD FUNCTION FOR GYROSCOPE
        break;
      case magCommand:
        // Get number of bytes associated with magnetometer data
        receiveData = receiveFromCarla(MAGBYTES);
        // Put received data into the magnetometer buffer
        loadMagBuffer(receiveData);
        break;
      case gpsCommand:
        // Get number of bytes associated with gps data
        receiveData = receiveFromCarla(GPSBYTES);
        // TODO: ADD FUNCTION FOR GPS
        break;
      default:
        // if unknown code, flush serial bus data to ensure we aren't out of sync.
        while (SerialUSB.available() > 0) {
          receiveData = receiveFromCarla(1);
        }
        break;
    }
  }




}

volatile byte * receiveFromCarla(int byteCount) {
  static volatile byte data[RECEIVEBUFFERSIZE];
  for (byte i = 0; i < byteCount; i++) {
    while (!SerialUSB.available()) {
      //wait for data to be available
    }
    data[i] = SerialUSB.read();
  }
  return data;
}
/*
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
*/

// Sends data to carla. Command list needs match command list on Carla side.
// Defined in router.h as an enumeration
void sendToCarla(carlaCommand mode, byte data[], int byteCount) {
  //SerialUSB.write(mode);
  char sendData[byteCount];
  for (byte i = 0; i < byteCount; i++) {
    sendData[i] = data[i];
  }

  SerialUSB.write(mode);
  SerialUSB.write(sendData, byteCount);
}

// Load the Accelerometer buffer from the data received from Carla.
void loadAccelBuffer(volatile byte receiveData[]) {
  for (byte i = 0; i < ACCELBYTES; i++) {
    accelDataBuffer[i] = receiveData[i];
  }
  // after buffer is loaded, inform high level data is ready
  SETACCELINTERRUPT;
}

//Load the Magnetometer buffer from the data received from Carla.
void loadMagBuffer(volatile byte receiveData[]) {
  for (byte i = 0; i < MAGBYTES; i++) {
    magDataBuffer[i] = receiveData[i];
  }
  // after buffer is loaded, inform high level data is ready
  SETMAGINTERRUPT;
}

//Blink the on-board LED to test the cyclometer
void Blink() {
  digitalWrite(LED_BUILTIN, digitalRead(LED_BUILTIN) ^ 1);
}



/****************************************************************

                          ISR ROUTINES

 ****************************************************************/

/* THE CYCLOMETER FUNCTION NEEDS TO BE REDEFINED TO MATCH THE NEW SCOPE 
 *  Sends pulse to low level for speed calculation. Currently on random digital pin.  Has to be
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


// Measured PWM frequency from Low level board is ~490Hz, So PWM signal is between 0 and 2040 us.
// Steering ISR, Interrupt on change.
void manageSteering() {

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
      steeringPrevious = steering; // used to check if steering has changed.
      pulseTime = ((volatile long)micros() - streerTimerStart);
      steering = map(pulseTime, 0, 2040, -128, 127);
      if (steering != steeringPrevious)
        inSteering = true;
      //restart the timer
      streerTimerStart = 0;
    }
  }

}


// ISR for brake pin
void manageBrake () {
  if (digitalRead(BRAKE_PIN) == HIGH) {
    brake = 0xFF;  // Feel free to change
    throttleSent = 0;
    inBraking = true;
  }
  else {
    brake = 0;
    inBraking = false;
  }
  brakeChange = true;
}

// ISR for read from Accelerometer address
void accelEvent() {
  Wire.write(accelDataBuffer, ACCELBYTES);
  CLEARACCELINTERRUPT;
}
// ISR for read from Magnetometer address
void magEvent() {
  Wire1.write(magDataBuffer, MAGBYTES);
  CLEARMAGINTERRUPT;
}
