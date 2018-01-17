/****************************************************************************
* Title                 :   Heart Rate 3 Application Layer
* Filename              :   heartrate_3.c
* Author                :   CAL
* Origin Date           :   05/11/2016
* Notes                 :   None
*****************************************************************************/
/**************************CHANGE LIST **************************************
*
*    Date     Software Version     Initials      Description
*  05/11/2016    XXXXXXXXXXX         CAL      Interface Created.
*
*****************************************************************************/
/** @file XXX.c
 *  @brief This module contains the 
 */
/******************************************************************************
* Includes
*******************************************************************************/
#include <Arduino.h>
#include "heartrate_3.h"
#include "heartrate_3_hw.h"


/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/

/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/

/******************************************************************************
* Module Typedefs
*******************************************************************************/

/******************************************************************************
* Module Variable Definitions
*******************************************************************************/

//TI HRM Library Variables start
unsigned long peakWindowHP[21], lastOnsetValueLED1, lastPeakValueLED1;
unsigned char HR[12],HeartRate,temp;
unsigned int lastPeak,lastOnset;
unsigned long movingWindowHP;
unsigned char ispeak=0;
unsigned char movingWindowCount, movingWindowSize, smallest, foundPeak, totalFoundPeak;
unsigned int frequency;
unsigned long currentRatio=0;
//TI HRM Library Variables end


/******************************************************************************
* Function Prototypes
*******************************************************************************/

/******************************************************************************
* Function Definitions
*******************************************************************************/

led_values_t* hr3_get_values( void )
{
    led_values_t led_values;

    led_values.led1_val = hr3_get_led1_val();
    led_values.led2_val = hr3_get_led2_val();
    led_values.led3_val = hr3_get_led3_val();

    return &led_values;
}


//TI HRM Library functions end


/*************** END OF FUNCTIONS ***************************************************************************/



