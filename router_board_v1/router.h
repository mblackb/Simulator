
#define BAUDRATE 115200


// Commands recognized by Carla - needs to match mapping on Carla.
enum carlaCommand {
  throttleCommand = 0x00,
  steerCommand,
  brakeCommand
};

// Define number of bytes to send for each command - needs to match number expected by Carla.
#define THROTTLEBYTES 1
#define BRAKEBYTES 1
#define STEERBYTES 1
