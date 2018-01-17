/*******************************************************************************
  Title                 :   HeartRate 3 click
  Filename              :   heartrate_3_hal.c
  Author                :   CAL
  Origin Date           :   04/06/2016
  Notes                 :   None
*******************************************************************************/
/*************** MODULE REVISION LOG ******************************************

     Date     Software Version    Initials      Description
   04/06/2016    XXXXXXXXXXX        CAL      Interface Created.

*******************************************************************************/
/**
   @file heartrate_3_hal.c
   @brief <h2> HAL layer </h2>
*/
/******************************************************************************
  Includes
*******************************************************************************/
#include <Arduino.h>
#include <Wire.h>
#include "heartrate_3_hal.h"
/******************************************************************************
  Module Preprocessor Constants
*******************************************************************************/

/******************************************************************************
  Module Preprocessor Macros
*******************************************************************************/

/******************************************************************************
  Module Typedefs
*******************************************************************************/

/******************************************************************************
  Module Variable Definitions
*******************************************************************************/
static uint8_t _i2c_hw_address;

/******************************************************************************
  Function Prototypes
*******************************************************************************/

/******************************************************************************
  Function Definitions
*******************************************************************************/
void hr3_hal_init( uint8_t address_id )
{

}

void hr3_hal_write( uint8_t *command,
                    uint8_t *buffer,
                    uint16_t count )
{
  uint8_t temp[ BUFF_SIZE ];

  uint8_t cmd_size    = 1;
  uint16_t i          = 0;
  uint8_t *temp_ptr   = temp;
  uint8_t *buff_ptr   = buffer;
  uint8_t *cmd_ptr    = command;

  /* Fill the temp buffer with data*/
  while ( cmd_size-- )
    temp[ i++ ] = *( cmd_ptr++ );

  while ( count-- )
    temp[ i++ ] = *( buff_ptr++ );

  //    Serial.print("write");
  Wire.beginTransmission(0x58);
  //    Serial.print(0x58);
  //    Serial.print(',');
  while ( i-- ) {
    //        Serial.print(*temp_ptr);
    //        Serial.print(',');
    Wire.write( *( temp_ptr++ ) );
  }
  Wire.endTransmission();    // stop transmitting
  //    Serial.println();
}


void hr3_hal_read( uint8_t *command,
                   uint8_t *buffer,
                   uint16_t count )
{
  uint8_t cmd_size = 1;
//  Serial.println("here");

  Wire.beginTransmission(0x58);
  while ( cmd_size-- ) {
//    Serial.println(*command);
    Wire.write( *( command++ ) );
  }
  Wire.endTransmission();

  Wire.requestFrom(0x58, count);    // request 6 bytes from slave device #8

  while (Wire.available() > 0) { // slave may send less than requested
    *( buffer++ ) = Wire.read(); // receive a byte as character
//    Serial.println(*buffer);
  }
  //    *buffer = Wire.read();
  //    Serial.println(*buffer);
  //    Serial.println("reading i2c");
}


void hr3_hal_delay( uint32_t ms )
{
  while ( ms--  )
    delay(1);
}

/*************** END OF FUNCTIONS *********************************************/
