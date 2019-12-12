'''
Repeatedly sends a single message to the CAN transceiver.
Heavily borrows from https://copperhilltech.com/blog/app-note-arduino-due-2channel-can-bus-driver-software/
'''

#include "DueCANLayer.h"

// CAN Layer functions
extern byte canInit(byte cPort, long lBaudRate);
extern byte canTx(byte cPort, long lMsgID, bool bExtendedFormat, byte* cData, byte cDataLen);
extern byte canRx(byte cPort, long* lMsgID, bool* bExtendedFormat, byte* cData, byte* cDataLen);

void setup()
{
  // Set the serial interface baud rate
  Serial.begin(115200);
  
  // Initialize CAN controller
  if(canInit(0, CAN_BPS_500K) == CAN_OK)
    Serial.print("CAN0 Initialized Successfully.\n\r");
  else
    Serial.print("CAN0 Initialization Failed.\n\r");
  
}// end setup

void loop()
{
  ////////////////////////////////////////////////////
  //              CAN Message sent                 //
  //---------------------------------------------- //
  // CHANGE THIS TO TEST DIFFERENT CAN MESSAGES   //
  /////////////////////////////////////////////////
  byte cTxData[] = {0x00, 0xFF, 0x00, 0x00, 0xFF, 0xEB, 0x00, 0x00};
  /////////////////////////////////////////////////////////////////
  int nTimer = 0;
  
  while(1)  // Endless loop
  {
    delay(1);
    
    // Send the message every 1000 milliseconds
    if(++nTimer == 1000)
    {
      if(canTx(0, 0x350, true, cTxData, 8) == CAN_OK)
        Serial.print("Data sent successfully.\n\r");
      else
        Serial.print("Error during data transmission.\n\r");
  
      nTimer = 0;

    }// end if

//    // Check for received message
//    long lMsgID;
//    bool bExtendedFormat;
//    byte cRxData[8];
//    byte cDataLen;
//    if(canRx(0, &lMsgID, &bExtendedFormat, &cRxData[0], &cDataLen) == CAN_OK)
//    {
//      Serial.print("Rx - MsgID:");
//      Serial.print(lMsgID, HEX);
//      Serial.print(" Ext:");
//      Serial.print(bExtendedFormat);
//      Serial.print(" Len:");
//      Serial.print(cDataLen);
//      Serial.print(" Data:");
//
//      for(byte cIndex = 0; cIndex < cDataLen; cIndex++)
//      {
//        Serial.print(cRxData[cIndex], HEX);
//        Serial.print(" ");
//      }// end for
//
//      Serial.print("\n\r");
//      
//    }// end if

  }// end while

}// end loop
