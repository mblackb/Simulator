#ifndef RC_RF69_H
#define RC_RF69_H

#define UART_BAUDRATE 115200
#define RF69_FREQ_MHZ 915.0

// The encryption key has to be the same for transmitter and receiver
//uint8_t key[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
//                 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00};

typedef union {
  struct
  {
    uint16_t throttle;
    uint16_t turn;
    uint8_t ebrake;
    uint8_t autonomous;
    uint8_t rssi; // "The current RSSI value in units of 0.5dB", negative
  };
  char bytes[10]; // The length of this byte buffer is dependded on the amount of Bytes in the struct above
} DataFromTransmitter;

typedef struct
{
  uint8_t rssi; // "The current RSSI value in units of 0.5dB", negative
} DataFromReceiver;

// transmitter input pins
#define TURN_PIN A3     // joystick 2 L/R
#define THROTTLE_PIN A2 // joystick 1 U/D
#define INTERRUPT_PIN 2
#define AUTO_PIN 5
#define EBRAKE_PIN 4
#define REVERSE_PIN 20

// ----- IMPORTED FROM THE Can_Protocal.h (path: elcano/Elcano_C2_lowlevel/Can_Protocal.h)---------
#define RCStatus_CANID 0x50
#define HiStatus_CANID 0x100
#define LowStatus_CANID 0x200
#define RCDrive_CANID 0x300
#define HiDrive_CANID 0x350
#define Actual_CANID 0x500
// ----- IMPORTED FROM THE Can_Protocal.h (path: elcano/Elcano_C2_lowlevel/Can_Protocal.h)---------



// LED pins on the transmitter
#define TX_LED_LINK 6

// other outputs
#define SS_PIN 10
/* pins 11 - 13 are reserved for SPI */

#endif
