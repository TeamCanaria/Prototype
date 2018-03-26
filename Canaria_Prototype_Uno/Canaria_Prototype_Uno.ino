#include <stdint.h>
#include <Wire.h>
#include <avr/interrupt.h>
#include <avr/io.h>


/************DEFINE******************/
#define POLL_PERIOD_US                      1E06 / 100    //10000us

/**************************************/

/************VARIABLES******************/
//const int LED_PIN = 4;
const int RED_LED = 16;
const int GREEN_LED = 5;
const int ADC_PIN = 17; // A3 on Arduino Nano
uint32_t tsLastPollUs = 0;
float volt = 0;
/**************************************/

/**********STRUCT*********************/
typedef struct {
    uint32_t led1;
    uint32_t led2;
    uint32_t led3;
} led_values_t;

led_values_t LedValues;
/*************************************/
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  
//  digitalWrite(RED_LED, 1);
}


uint32_t get_led_val(uint8_t adcPin) {
  uint32_t retval = 0;
  retval = analogRead(adcPin);
  return retval;
}

void powerSequence (void) {
  /*LED2 starts and ends - 100us*/
  digitalWrite(RED_LED, 0);
//  delayMicroseconds(75);
  /* LED2 starts conversion*/
  LedValues.led1 = get_led_val(ADC_PIN);
//  digitalWrite(RED_LED, 1);
//  delayMicroseconds(270);
}
/*******Main loop********/
void loop() {
  // put your main code here, to run repeatedly:
  
  powerSequence();
  

  if (micros() < tsLastPollUs || micros() - tsLastPollUs > POLL_PERIOD_US) {
        
        tsLastPollUs = micros();        
        Serial.println(LedValues.led1);
//          Serial.println("Hello");
//        volt = (5.0/1023.0)*(LedValues.led1);
//        Serial.println(volt);
    }
//  delay(100);
}
