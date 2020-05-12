//#include "peripherals\L3GD20H-Gyro.h"
//#include "peripherals\FGPMMOPA6H-GPS.h"
//#include "peripherals\LSM303DLHC-Accel-Mag.h"


// For UART connections
#define BAUDRATE            115200
#define RECEIVEBUFFERSIZE   32  // needs to be equal to or greater than highest number of receive bytes.
#define GPSSERIAL           Serial3
#define GPSBAUD             9600


// Define pins used for functions
#define PULSE_PIN           5
#define BRAKE_PIN           9
#define STEER_PIN           6
#define THROTTLE_PIN        A1

// Accelerometer and Magnetometer
#define ACCELADDRESS        0x19
#define MAGADDRESS          0x1E
#define ACCELINT            27
#define MAGINT              28

// interrupt macros
#define SETACCELINTERRUPT   digitalWrite(ACCELINT,LOW);
#define CLEARACCELINTERRUPT digitalWrite(ACCELINT,HIGH);
#define SETMAGINTERRUPT     digitalWrite(MAGINT,LOW);
#define CLEARMAGINTERRUPT   digitalWrite(MAGINT,HIGH);

// Commands recognized by Carla - needs to match mapping on Carla.
enum carlaCommand {
  throttleCommand = 0x00, // 0x00
  steerCommand,           // 0x01
  brakeCommand,           // 0x02
  accelCommand,           // 0x03
  gyroCommand,            // 0x04
  magCommand,             // 0x05
  gpsCommand              // 0x06
};

// Define number of bytes to send for each command - needs to match number expected by Carla.
#define THROTTLEBYTES       1
#define BRAKEBYTES          1
#define STEERBYTES          1   
#define ACCELBYTES          6   // [xx,yy,zz]
#define GYROBYTES           1
#define MAGBYTES            6   // [xx,yy,zz]
#define GPSBYTES            21  // lat[9],latdir,long[10],longdir (ddmm.mmmmNdddmm.mmmmW) i.e: 4559.4810N12269.3800W


#define THROTTLENOISEMASK   5
