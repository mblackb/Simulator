#include <Wire.h>
extern TwoWire Wire1;
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_LSM303_Accel.h>
#include <Adafruit_LSM303DLH_Mag.h>
#include <Adafruit_L3GD20.h>



/* Assign a unique ID to each sensor at the same time */
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_L3GD20 gyro = Adafruit_L3GD20(13579);

void setup(void)
{
    //the yall use this baud rate
    Serial.begin(115200);

    //autograins the magnetometer
    mag.enableAutoRange(true);
    

    Serial.println("Sensors Test"); Serial.println("");

    //checks if any sensor fails to instantiate
    if (!mag.begin() && !accel.begin() && !gyro.begin(gyro.L3DS20_RANGE_250DPS)) {
        Serial.println("Ooops, a sensor wasn't detected ... Check your wiring!");
        while (1);
    }

    //prints the pertinent sensor data
    displaySensorDetails();


    //sets the accelerometer to +-4 gravities reading
    accel.setRange(LSM303_RANGE_4G);
    lsm303_accel_range_t new_range = accel.getRange();

    Serial.print("Range is ");
    Serial.println(new_range);

    //normal reading samples data at 1.344 kHz
    //this is a bit slower but is high-precision
    accel.setMode(LSM303_MODE_NORMAL);

    //normal reading samples data at 5.376 kHz
    //reads much faster, but with less precision than normal mode
    //accel.setMode(LSM303_MODE_NORMAL);

    lsm303_accel_mode_t newMode = accel.getMode();
    Serial.print("Mode is ");
    Serial.println(newMode);
}

void loop(void)
{
    //sensors_event_t is used to abstract any type of sensor data into a single object
    //there is no need to create a new sensors_event_t for each sensor
    //after calling, event object saves the data and configures automatically
    sensors_event_t event;

    //pass by reference since each sensor_object is 36 bytes each
    //saves on memory, and many data types of sensor data are interpreted by this object
    //as long as it is an adafruit object using it
    mag.getEvent(&event);

    
    float heading = (atan2(event.magnetic.y, event.magnetic.x) * 180) / PI;
    
    //Normalize to 0-360
    if (heading < 0)
    {
        heading = 360 + heading;
    }


    //recall getEvent to replace with accelerometer data
    accel.getEvent(&event);
    xAccel = event.acceleration.x;
    yAccel = event.acceleration.y;
    zAccel = event.acceleration.y;

    //instantiates the gyroscope object
    gyro.read();

    //print to console the x, y, z gyroscopic data
    Serial.print("X: "); Serial.print((int)gyro.data.x);   Serial.print(" ");
    Serial.print("Y: "); Serial.print((int)gyro.data.y);   Serial.print(" ");
    Serial.print("Z: "); Serial.println((int)gyro.data.z); Serial.print(" ");

    Serial.print("Acceleration: ");
    Serial.println(xAccel + "  " + yAccel + " " + zAccel + " m/s^2");
    Serial.print("Compass Heading: ");
    Serial.println(heading);
    delay(5000);
}

void displaySensorDetails(void) {
    sensor_t sensor;
    accel.getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.print("Sensor:       ");
    Serial.println(sensor.name);
    Serial.print("Driver Ver:   ");
    Serial.println(sensor.version);
    Serial.print("Unique ID:    ");
    Serial.println(sensor.sensor_id);
    Serial.print("Max Value:    ");
    Serial.print(sensor.max_value);
    Serial.println(" m/s^2");
    Serial.print("Min Value:    ");
    Serial.print(sensor.min_value);
    Serial.println(" m/s^2");
    Serial.print("Resolution:   ");
    Serial.print(sensor.resolution);
    Serial.println(" m/s^2");
    Serial.println("------------------------------------");
    Serial.println("");
    delay(5000);

    sensor_t sensor1;
    mag.getSensor(&sensor1);
    Serial.println("------------------------------------");
    Serial.print("Sensor:       ");
    Serial.println(sensor1.name);
    Serial.print("Driver Ver:   ");
    Serial.println(sensor1.version);
    Serial.print("Unique ID:    ");
    Serial.println(sensor1.sensor_id);
    Serial.print("Max Value:    ");
    Serial.print(sensor1.max_value);
    Serial.println(" uT");
    Serial.print("Min Value:    ");
    Serial.print(sensor1.min_value);
    Serial.println(" uT");
    Serial.print("Resolution:   ");
    Serial.print(sensor1.resolution);
    Serial.println(" uT");
    Serial.println("------------------------------------");
    Serial.println("");
}