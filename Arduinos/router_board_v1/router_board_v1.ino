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

  Portions of GPS code based on information found here: https://ucexperiment.wordpress.com/2012/03/12/arduino-scripted-gps-simulator/

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

// Accelerometer and Magnetometer buffers
byte accelDataBuffer[ACCELBYTES];
byte magDataBuffer[MAGBYTES];

// Magnetometer flags
bool magRegMRead;
bool magRegMgRead;

// GPS Variables
char sec[2];
char lat[10];
char latdir[2];
char lng[11];
char lngdir[2];
char hdg[7];
char spd[7];
volatile uint8_t t4 = 10;
volatile uint8_t t3 = 30;
volatile uint8_t t2 = 0;
volatile uint8_t t1 = 0;
volatile boolean _1Hz_flag = 0;
char gprmc[96];


//Set pins, prepare serial.
void setup() {

  // Serial port used for debug
  Serial.begin(BAUDRATE);
  // Serial USB port used for Carla connection
  SerialUSB.begin(BAUDRATE);
  // Serial port for GPS
  GPSSERIAL.begin(GPSBAUD);

  //GPS variables initial loading
  _1Hz_flag = false;
  sec[0] = '0';
  sec[1] = '\0';
  lat[10] = '\0';
  latdir[1] = '\0';
  lng[11] = '\0';
  lngdir[1] = '\0';
  hdg[7] = '\0';
  spd[7] = '\0';


  // Built in built in LED for debug
  pinMode(LED_BUILTIN, OUTPUT);

  // Set pins
  pinMode(BRAKE_PIN, INPUT);
  pinMode(STEER_PIN, INPUT);

  // Start cyclometer pulse with arbitrarily large value
  Timer3.attachInterrupt(sendPulse).start(2000000);

  // Setup I2C for accelerometer and Magnetometer
  // Currently only Magnetrometer is setup for heading data to high level
  // Need to develop emulation of multiple slave addresses on single bus.
  //Wire.begin(ACCELADDRESS);
  //Wire.onRequest(accelEvent);
  Wire.begin(MAGADDRESS);       // Accelerometer and magnetometer addresses
  Wire.onRequest(magEvent);  // register callback function for sending magnetometer data
  Wire.onReceive(magRegEvent); // Register callback for receiving commands for magnetometer

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
  - Setup Magnetometer so it can be read properly

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

  // Receive data from Carla
  if (SerialUSB.available()) {
    volatile byte *receiveData = receiveFromCarla(1);
    Serial.println(receiveData[0]); //debug
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

        uint8_t i;
        /* No need to send different milliseconds.
                //time
                sec[0] = receiveData[0];
        */
        //latitude
        for (i = 0; i < 9; i++)
          lat[i] = receiveData[i];
        latdir[0] = receiveData[9];
        //longitude
        for (i = 0; i < 10; i++)
          lng[i] = receiveData[i + 10];
        lngdir[0] = receiveData[20];
        /* Not currently using speed or heading from GPS
                //speed
                for (i = 0; i < 6; i++)
                  spd[i] = receiveData[i + 20];
                //heading
                for (i = 0; i < 6; i++)
                  hdg[i] = receiveData[i + 26];
        */
        //Data read to send flag
        _1Hz_flag = true;
        break;
      default:
        // if unknown code, flush serial bus data to ensure we aren't out of sync.
        while (SerialUSB.available() > 0) {
          receiveData = receiveFromCarla(1);
        }
        break;
    }

    //Send GPS data to high level board if it is available
    if (_1Hz_flag) {
      IncrementTime();
      //format $GPRMC sentence
      // Message format is: UMT time, A, Lat, Long, Speed, Heading, Date, Magnetic variation*Checksum
      sprintf(gprmc, "$GPRMC,%2.2u%2.2u%2.2u.000,A,%9s,%s,%10s,%s,000.0,000.0,080112,,S", t4, t3, t2, lat, latdir, lng, lngdir);
      //calculate and add checksum
      calcGPSCheckSum(gprmc);
      // Send for debug
      Serial.println(gprmc);
      // Send GPS data to high Level
      GPSSERIAL.println(gprmc);
      _1Hz_flag = false;
    }
  }

//test for seeing loaded magnetometer data
//static long test=0;
//
//if (test%10000==1){
//SerialUSB.print("Mag Data: ");
//SerialUSB.print(magDataBuffer[0]);
//SerialUSB.print(magDataBuffer[1]);
//SerialUSB.print(magDataBuffer[2]);
//SerialUSB.print(magDataBuffer[3]);
//SerialUSB.print(magDataBuffer[4]);
//SerialUSB.println(magDataBuffer[5]);
//}
//
//test++;

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

                          GPS Functions

 ****************************************************************/

void IncrementTime() {

  t2++;
  if (t2 > 59) {
    t2 = 0;
    t3++;
  }
  if (t3 > 59) {
    t3 = 0;
    t4++;
  }
  if (t4 > 23)
    t4 = 0;
}

void calcGPSCheckSum(char *buff)
{
  char cs = 0;
  int i = 1;
  while (buff[i]) {
    cs ^= buff[i];
    i++;
  }
  sprintf(buff, "%s*%02X", buff, cs);
}

/****************************************************************

                          ISR ROUTINES

 ****************************************************************/

/* THE CYCLOMETER FUNCTION NEEDS TO BE REDEFINED TO MATCH THE NEW SCOPE
    Sends pulse to low level for speed calculation. Currently on random digital pin.  Has to be
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

/*
// ISR for read from Accelerometer address
void accelEvent() {
  Wire.write(accelDataBuffer, ACCELBYTES);
  CLEARACCELINTERRUPT;
}
*/

// ISR for read from Magnetometer address
void magEvent() {
  if (magRegMRead) {
    Wire.write(0x10);  // Adafruit_LSM303_U driver expects to read 0x10 from this register
  } else if (magRegMgRead) {
    Wire.write(0x01);  // send data ready bit
  } else {
    Wire.write(magDataBuffer, MAGBYTES);
    CLEARMAGINTERRUPT;
  }
}

void magRegEvent(int howMany) {
  char reg = Wire.read();
      //SerialUSB.println(reg); // for debug
  switch (reg) {
    case LSM303_REGISTER_MAG_CRA_REG_M:
      magRegMRead = true;
      break;
    case LSM303_REGISTER_MAG_SR_REG_Mg:
      magRegMgRead = true;
      break;
    default:
      break;
  }
  while (Wire.available()) // clear buffer if this is a write command
  {
    char c = Wire.read(); // receive byte as a character
    //clear read flags because this is a write command
    magRegMRead = false;
    magRegMgRead = false;
  }

}
