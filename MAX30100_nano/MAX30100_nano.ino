/*
Arduino-MAX30100 oximetry / heart rate integrated sensor library
Copyright (C) 2016  OXullo Intersecans <x@brainrapers.org>
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// The example shows how to retrieve raw values from the sensor
// experimenting with the most relevant configuration parameters.
// Use the "Serial Plotter" app from arduino IDE 1.6.7+ to plot the output

#include <Wire.h>
#include <SoftwareSerial.h>
#include "MAX30100.h"

// Tweakable parameters
// Sampling and polling frequency must be set consistently
#define POLL_PERIOD_US                      1E06 / 100
#define SAMPLING_RATE                       MAX30100_SAMPRATE_100HZ

// The LEDs currents must be set to a level that avoids clipping and maximises the
// dynamic range
#define IR_LED_CURRENT                      MAX30100_LED_CURR_50MA
#define RED_LED_CURRENT                     MAX30100_LED_CURR_27_1MA

// The pulse width of the LEDs driving determines the resolution of
// the ADC (which is a Sigma-Delta).
// set HIGHRES_MODE to true only when setting PULSE_WIDTH to MAX30100_SPC_PW_1600US_16BITS
#define PULSE_WIDTH                         MAX30100_SPC_PW_1600US_16BITS
#define HIGHRES_MODE                        true


// Instantiate a MAX30100 sensor class
MAX30100 sensor;
uint32_t tsLastPollUs = 0;

int dc_remove_IR = 0;
int mean_median_IR = 0;
int lp_butterworth_IR = 0;
int old_lp_butterworth = 0;
int sample = 0;
int last_sample = 0;

SoftwareSerial BTserial(2, 3); // RX | TX
// Connect the HC-06 TX to the Arduino RX on pin 2. 
// Connect the HC-06 RX to the Arduino TX on pin 3 through a voltage divider.



struct meanDiffFilter_t
{
  float values[15];
  byte index;
  float sum;
  byte count;
} filterValues;

void setup()
 {
    Serial.begin(115200);
    Serial.print("Initializing MAX30100..");
    Serial.println("Enter AT commands:");

    BTserial.begin(9600);

    // Initialize the sensor
    // Failures are generally due to an improper I2C wiring, missing power supply
    // or wrong target chip
    if (!sensor.begin()) {
        Serial.println("FAILED");
        for(;;);
    } else {
        Serial.println("SUCCESS");
    }

    // Set up the wanted parameters
    sensor.setMode(MAX30100_MODE_SPO2_HR);
    sensor.setLedsCurrent(IR_LED_CURRENT, RED_LED_CURRENT);
//    sensor.setLedsCurrent(IR_LED_CURRENT, 0x00);
//    sensor.setLedsCurrent(0x00, 0x00);
    sensor.setLedsPulseWidth(PULSE_WIDTH);
    sensor.setSamplingRate(SAMPLING_RATE);
    sensor.setHighresModeEnabled(HIGHRES_MODE);
}

float meanDiff(float M, meanDiffFilter_t* filterValues)
{ 
  float avg = 0;

  filterValues->sum -= filterValues->values[filterValues->index];
  filterValues->values[filterValues->index] = M;
  filterValues->sum += filterValues->values[filterValues->index];



  filterValues->index++;
  filterValues->index = filterValues->index % 15;

  if(filterValues->count < 15)
    filterValues->count++;

  avg = filterValues->sum / filterValues->count;
  return avg - M;
}

void loop()
 {

    // Using this construct instead of a delay allows to account for the time
    // spent sending data thru the serial and tighten the timings with the sampling
    last_sample = sample;
    if (micros() < tsLastPollUs || micros() - tsLastPollUs > POLL_PERIOD_US) {
        sensor.update();
        tsLastPollUs = micros();
        sample = -(sensor.rawIRValue);
        dc_remove_IR = (sample + 0.988 * dc_remove_IR) - last_sample;
        mean_median_IR = meanDiff(dc_remove_IR, &filterValues);
        lp_butterworth_IR = (2.452372752527856026e-1 * mean_median_IR) + (0.50952544949442879485 * old_lp_butterworth);
        old_lp_butterworth = lp_butterworth_IR;
        Serial.println(sample);
    }
}

  // Keep reading from HC-06 and send to Arduino Serial Monitor
  if (BTserial.available())
  {  
    Serial.write(BTserial.read());
   }
 
  // Keep reading from Arduino Serial Monitor and send to HC-06
  if (Serial.available())
  {
    BTserial.write(Serial.read());
   }
  
  last_sample = sample;
  if (micros() < tsLastPollUs || micros() - tsLastPollUs > POLL_PERIOD_US) {
    sensor.update();
    tsLastPollUs = micros();
    sample = -(sensor.rawIRValue);
    //sample = sensor.rawRedValue;
    // Apply Dc Remove
    //dc_remove_IR = (sample + 0.9666 * dc_remove_IR) - last_sample;
    // Apply Mean Median Filter
    mean_median_IR = meanDiff(dc_remove_IR, &filterValues);
    // Apply Low-pass Butterworth Filter
    //lp_butterworth_IR = (2.452372752527856026e-1 * mean_median_IR) + (0.50952544949442879485 * old_lp_butterworth);
    //old_lp_butterworth = lp_butterworth_IR;
    //Serial.println(sample); 
    Serial.println(sample);
    }
   
}

