//#include "peripherals\L3GD20H-Gyro.h"
//#include "peripherals\FGPMMOPA6H-GPS.h"
//#include "peripherals\LSM303DLHC-Accel-Mag.h"


// For UART connections
#define BAUDRATE            115200
#define RECEIVEBUFFERSIZE   10  // needs to be equal to or greater than highest number of receive bytes.

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
  throttleCommand = 0x00,
  steerCommand,
  brakeCommand,
  accelCommand,
  gyroCommand,
  magCommand,
  gpsCommand
};

// Define number of bytes to send for each command - needs to match number expected by Carla.
#define THROTTLEBYTES       1
#define BRAKEBYTES          1
#define STEERBYTES          1
#define ACCELBYTES          6
#define GYROBYTES           1
#define MAGBYTES            6
#define GPSBYTES            2


#define THROTTLENOISEMASK   5
