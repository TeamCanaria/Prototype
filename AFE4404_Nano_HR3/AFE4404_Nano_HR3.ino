#include <stdint.h>
#include <Wire.h>
#include "heartrate_3.h"



  
const int AFE_RST_PIN = 3; //12  
const int AFE_RDY_PIN = 2; //14
const int LED_PIN = 4;

long timeStamp = millis();
volatile bool isAdcReady = false;


/********************Setup AFE and Interrupt****************/
void setup()
{
  Serial.begin(115200);
  Serial.println("UART Initialized");

  pinMode(LED_PIN, OUTPUT);     // External LED for testing Interrupt
  pinMode(AFE_RDY_PIN, INPUT_PULLUP); // Declare the interrupt pin (pin 2)
  
  AFE4404Setup();     // GPIO / HeartRate 3 / UART / I2C Setups

  interruptSetup();
  delay(200);
}

/**********Main LOOP function*****************/
void loop(){
  if(isAdcReady){
    led_values_t * vals = hr3_get_values();   // Get values from photodiode (Red-Green-Infar)
//    statHRMAlgo( (*vals).led1_val );
    timedEvents();        // Counter for the delay
    isAdcReady = false;   // Finish

    Serial.println(vals->led2_val);
  } 
}


/**************Counter for delay time**************/
void timedEvents(){
  if(millis() - timeStamp > 200){
  
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    timeStamp = millis();
  }  
}






/*************AFE setup function************/
void AFE4404Setup( void ){
  //Local Declarations
  char            text[20] = { 0 };   // Unused
  dynamic_modes_t dynamic_modes;      // Structure contains.....
  uint8_t         address = 0x58;

  //  RST_DIR = 1;
  pinMode(AFE_RST_PIN, OUTPUT);       // Declare the pin reset of Arduino Nano
  digitalWrite(AFE_RST_PIN, HIGH);    // Hardware reset

  //Set up dynamic modes for Heart Rate 3 Initialization
  // 
  dynamic_modes.transmit = trans_en;                       //Transmitter disabled
  dynamic_modes.curr_range = led_norm;                     //LED range 0 - 100
  dynamic_modes.adc_power = adc_on;                        //ADC on
  dynamic_modes.clk_mode = osc_mode;                       //Use internal Oscillator
  dynamic_modes.tia_power = tia_off;                       //TIA off
  dynamic_modes.rest_of_adc = rest_of_adc_off;             //Rest of ADC off
  dynamic_modes.afe_rx_mode = afe_rx_normal;               //Normal Receiving on AFE
  dynamic_modes.afe_mode = afe_normal;                     //Normal AFE functionality

  // TWI
//  Wire.begin(4, 5);

  Wire.begin();        //I2C setup
  delay(500);
  Serial.println( "TWI Initialized" );

  //Toggle Reset pin
  digitalWrite(AFE_RST_PIN, LOW);
  delayMicroseconds(50);
  digitalWrite(AFE_RST_PIN, HIGH);

  //Heart Rate 3 Initialize
  hr3_init( address, &dynamic_modes );
  Serial.println( "HR3 Initialized" );

//  initStatHRM();
  delay(100);
}

void interruptSetup( void ){
  attachInterrupt(digitalPinToInterrupt(AFE_RDY_PIN), adc_rdy_isr, RISING);
}

void adc_rdy_isr(){
    
  isAdcReady = true;

}

