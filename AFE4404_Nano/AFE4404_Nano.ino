#ifdef ARDUINO
#include <Wire.h>
#endif

#include "AFE4404.h"
#include "mbino.h"

// DIGITAL OUTPUTs
#define PIN_RST   D3

#define PIN_DRDY  D2
#define PIN_CLK   D5
#define I2C_SDA   D18 
#define I2C_SCL   D19

#define SET_BAUD_RATE 115200
#define DEBUG 0

AFE4404 afe(PIN_RST, PIN_DRDY, PIN_CLK , I2C_SDA, I2C_SCL);

RawSerial pc(USBTX, USBRX);
extern int32_t data;
extern bool dataAvailable;
volatile bool isAdcReady = false;
long timeStamp = millis();
char LED = 0x2A;
void powerUpSequence(void);

typedef struct {
    uint32_t led1;
    uint32_t led2;
    uint32_t led3;
} led_values_t;

led_values_t LedValues;

void adc_rdy_isr(){
  isAdcReady = true;
}

void powerUpSequence(void) {
  afe.writeData(0x00, 0x08);
  delay(500);
  pc.printf( "TWI Initialized\n" );
  
  /* Toggle Reset Pin */
  digitalWrite(PIN_RST, LOW);
  delayMicroseconds(50);
  digitalWrite(PIN_RST, HIGH);

  /* Heart Rate 3 Initialisation */
  afe.initRegisters();
  pc.printf( "HR3 Initialised" );
  delay(100);
}

uint32_t hr3_get_led_val(uint8_t reg) {
  uint8_t temp[3] = { 0 };
  uint32_t retval = 0;
  
  Wire.beginTransmission(0x58);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(0x58, 3);    // request 6 (3?) bytes from slave device #8

  for (uint8_t i = 0; i < 3; i++) {
    if (Wire.available() <= 0) { // Slave sent less than requested.
      break;
    }
    temp[i] = Wire.read();
  }
  
  retval |= (uint32_t)temp[0] << 16;
  retval |= (uint32_t)temp[1] << 8;
  retval |= (uint32_t)temp[2];
  
  return retval; // 2's Complement.
}

/*
 * Calls initialisation functions.
 */
void setup() {
  pc.printf("Setting up....\n");
  pc.baud(SET_BAUD_RATE);
  
  powerUpSequence();
  attachInterrupt(digitalPinToInterrupt(PIN_DRDY), adc_rdy_isr, RISING);
}

/*
 * Main active loop.
 */
void loop() {
  if(isAdcReady){
    LedValues.led1 = hr3_get_led_val(AFE4404_LED1VAL);
    LedValues.led2 = hr3_get_led_val(AFE4404_LED2VAL);
    LedValues.led3 = hr3_get_led_val(AFE4404_ALED2VAL_LED3VAL);

    pc.printf("%d\n", LedValues.led2);
    // pc.printf("%06X\n", LedValues.led2);
    
    if(millis() - timeStamp > 200){
      interrupts();    
      timeStamp = millis();
    }
    
    isAdcReady = false;
  } 
}
