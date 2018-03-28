#include <stdint.h>
#include <Wire.h>
#include <avr/interrupt.h>
#include <avr/io.h>


/************DEFINE******************/
#define POLL_PERIOD_US                      1E06 / 100    //10000us

/**************************************/

/************VARIABLES******************/
//const int LED_PIN = 4;
//const int RED_LED = 4;
const int RED_LED = 16; // A2 on Ardunio Nano
const int IR_LED = 15; //A1 on Arduino Nano

const int GREEN_LED = 5;
const int ADC_PIN = 17; // A3 on Arduino Nano

//const int BUTTON_INT = 14; // A0 on Ardunio Nano
const int BUTTON_INT = 2; // D2 on Ardunio Nano

uint32_t tsLastPollUs = 0;
float volt = 0;
volatile bool buttonPressed = false;
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
  pinMode(IR_LED, OUTPUT);
  

  pinMode(BUTTON_INT, INPUT);

  attachInterrupt(digitalPinToInterrupt(BUTTON_INT), adc_rdy_isr, RISING);
  
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

}
/*******Main loop********/
void loop() {
  // put your main code here, to run repeatedly:
  
  powerSequence();
  
  
  if (micros() < tsLastPollUs || micros() - tsLastPollUs > POLL_PERIOD_US) {
        
        tsLastPollUs = micros();        
        Serial.println(LedValues.led1);

//        volt = (5.0/1023.0)*(LedValues.led1);
//        Serial.println(volt);
    }
}

void adc_rdy_isr(){
  Serial.println("F");
  delay(1);
  buttonPressed = true;
}

