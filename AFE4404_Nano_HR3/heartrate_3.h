/****************************************************************************
* Title                 :   Heart Rate 3 Application Layer
* Filename              :   heartrate_3.h
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
/** @file heartrate_3.h
 *  @brief This is the application layer for the Heart Rate 3 click board.
 *
 *  @date 11 Apr 2016
 *  @author Corey Lakey
 *  @copyright GNU Public License
 *
 *  @version .1 - Initial testing and verification
 *
 *  @note Test configuration:
 *   MCU:             STM32F107VC
 *   Dev.Board:       EasyMx Pro v7
 *   Oscillator:      72 Mhz internal
 *   Ext. Modules:    GPS Click
 *   SW:              ARM 4.5.2
 *
 */
#ifndef HEARTRATE_3_H_
#define HEARTRATE_3_H_

/******************************************************************************
* Includes
*******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <Arduino.h>
#include "heartrate_3_hw.h"

/******************************************************************************
* Preprocessor Constants
*******************************************************************************/

/******************************************************************************
* Configuration Constants
*******************************************************************************/


/******************************************************************************
* Macros
*******************************************************************************/


        
/******************************************************************************
* Typedefs
*******************************************************************************/
typedef struct
{
    uint32_t led1_val;
    uint32_t led2_val;
    uint32_t led3_val;
}led_values_t;

/******************************************************************************
* Variables
*******************************************************************************/

/******************************************************************************
* Function Prototypes
*******************************************************************************/
#ifdef __cplusplus
extern "C"{
#endif

/**
 * @brief <h3> Get Values </h3>
 *
 * @par
 *  Gets all three values from the ADC ( LED1, LED2, LED3 ) and puts
 * them into a struct.
 *
 *
 * @retval LED Values
 *
 */
led_values_t* hr3_get_values( void );

/**
 * @brief <h3> Get HeartRate </h3>
 *
 * @par
 *  Uses heartrate monitor functions to produce a bpm for user to use.
 *
 *
 * @retval Heart rate
 *
 */


/**********************************/



#ifdef __cplusplus
} // extern "C"
#endif

#endif /*HEARTRATE_3_H_*/

/*** End of File **************************************************************/
