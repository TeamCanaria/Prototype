#ifdef ARDUINO
#include <Wire.h>
#endif
//#include <Wire.h>
#include "AFE4404.h"
//#include "i2c_t3.h"
#include "mbed.h"


//char LED = 0x2A;
int32_t data;
volatile bool dataAvailable = false;


//AFE4404::AFE4404(PinName rxSupplyEn, PinName txSupplyEn, PinName resetz, PinName powerEn, PinName drdy, PinName clk, PinName sda, PinName scl):
//
//  _rxSupplyEn(rxSupplyEn), _txSupplyEn(txSupplyEn), _resetz(resetz),
//  _powerEn(powerEn), _drdy(drdy), _clk(clk), _i2c(sda, scl) {
//    
//  _address = (0x58 << 1);
//}
AFE4404::AFE4404(PinName resetz, PinName drdy, PinName clk, PinName sda, PinName scl):

  _resetz(resetz),
  _drdy(drdy), _clk(clk), _i2c(sda, scl){

  _address = (0x58 << 1);
}

void AFE4404::initPorts(void) {

  // turn off power supplies
//  _rxSupplyEn   = 0;
//  _txSupplyEn   = 0;
//  _powerEn      = 0;

  // resetz is active low, so leave on before power supply init
  _resetz     = 1;

  // set the clock output to zero before power-up sequence
  // this convoluted method was required because of the way the the PWM
  // output is set up (faster that possible with the MBED APIs)
  _clk.period(10);
  _clk.write(0);

  disableIRQ();
}

void AFE4404::initPowerSupply(void) {

  wait_ms(100);

//  _powerEn = 1;
//  wait_ms(100);

//  _rxSupplyEn = 1;
//  wait_ms(10);

//  _txSupplyEn = 1;
//  wait_ms(20);

  _resetz = 0;
  wait_us(35);

  _resetz = 1;

  initClock();
  wait_ms(2);
}


uint32_t AFE4404::readData(uint8_t reg, bool adc = true) {

  if (!adc) {
    enableReadMode();
  }

  _writeBuffer[0] = reg;  // initialize write buffer with AFE register address

  // initialize read buffers to 0. probably unnecessary
  _readBuffer[0] = 0x00;
  _readBuffer[1] = 0x00;
  _readBuffer[2] = 0x00;

  // write the register to AFE and use repeated start mode as specified in
  // the datasheet
  _i2c.write(_address, _writeBuffer, 1, true);
  // read 3 bytes of data from register MSB first
  _i2c.read(_address, _readBuffer, 3);

  _tempData = 0;
  _tempData = (_readBuffer[0] << (BITS_PER_BYTE * 2)) | (_readBuffer[1] << BITS_PER_BYTE) | _readBuffer[2];

  if (adc && (SIGN_MASK & _tempData)) {
    _tempData |= SIGN_EXT;
  }

  return _tempData;

}

void AFE4404::writeData(uint8_t reg, uint32_t data) {

  enableWriteMode();

  _writeBuffer[0] = reg;

  // store the lower 3 bytes of data in _writeBuffer (MSB first)
  for (int i = 2, j = 1; i >= 0; i--, j++) {
    _writeBuffer[j] = (data >> (BITS_PER_BYTE * i)) & LOWER_BYTE_MASK;
  }

  // write 4 bytes
  // 1 for the register address and 3 for the lower 3 bytes of data
  _i2c.write(_address, _writeBuffer, 4);

}


struct Register {
  uint8_t addr;
  uint32_t val;
};

void AFE4404::initRegisters(void) {

  unsigned char i;
  struct Register reg[NUM_REGISTERS];
  //Table 5 in AFE4404 datasheet
  //Sample LED 2 start and LED 2 end
  reg[0].addr = AFE4404_LED2STC; reg[0].val = 0x000050; // 80
  reg[1].addr = AFE4404_LED2ENDC; reg[1].val = 0x00018F; //399

  //LED 1 start and LED1 end
  reg[2].addr = AFE4404_LED1LEDSTC; reg[2].val = 0x000320; //800
  reg[3].addr = AFE4404_LED1LEDENDC; reg[3].val = 0x0004AF; //1199 

  //Sample ambient  2 (or sample LED3) start
  reg[4].addr = AFE4404_ALED2STC_LED3STC ; reg[4].val = 0x0001E0; //480
  //Sample ambient 2 (or sample LED3) end
  reg[5].addr = AFE4404_ALED2ENDC_LED3ENDC; reg[5].val = 0x00031F; //799

  //Sample LED1 start
  reg[6].addr = AFE4404_LED1STC; reg[6].val = 0x000370; //880
  //Sample LED1 end
  reg[7].addr = AFE4404_LED1ENDC; reg[7].val = 0x0004AF; //1199

  //LED 2 start
  reg[8].addr = AFE4404_LED2LEDSTC; reg[8].val = 0x000000; //0
  //LED 2 end
  reg[9].addr = AFE4404_LED2LEDENDC; reg[9].val = 0x00018F; // 399

  // Sample ambient 1 start
  reg[10].addr = AFE4404_ALED1STC; reg[10].val = 0x0004FF; //1279
  //Sample ambient 1 end
  reg[11].addr = AFE4404_ALED1ENDC; reg[11].val = 0x00063E; // 1598

  //LED2 convert phase start
  reg[12].addr = AFE4404_LED2CONVST; reg[12].val = 0x000198; //408 
  // LED 2 convert phase end
  reg[13].addr = AFE4404_LED2CONVEND; reg[13].val = 0x0005BB; //1467

  //Ambient 2 (or LED3) convert phase start
  reg[14].addr = AFE4404_ALED2CONVST_LED3CONVST; reg[14].val = 0x0005C4; // 1476
  //Ambient 2 (or LED3) convert phase end
  reg[15].addr = AFE4404_ALED2CONVEND_LED3CONVEND; reg[15].val = 0x0009E7; //2535

  //LED 1 convert phase start
  reg[16].addr = AFE4404_LED1CONVST; reg[16].val = 0x0009F0; // 2544
  //LED 1 convert phase end
  reg[17].addr = AFE4404_LED1CONVEND; reg[17].val = 0x000E13; // 3603

  //Ambient 1 convert phase start
  reg[18].addr = AFE4404_ALED1CONVST ; reg[18].val = 0x000E1C; //3612
  //Ambient 1 convert phase end
  reg[19].addr = AFE4404_ALED1CONVEND; reg[19].val = 0x00123F; //4671

  //ADC reset phase 0 start and end
  reg[20].addr = AFE4404_ADCRSTSTCT0; reg[20].val = 0x000191; //401
  reg[21].addr = AFE4404_ADCRSTENDCT0; reg[21].val = 0x000197; // 407

  //ADC reset phase 1 start and end
  reg[22].addr = AFE4404_ADCRSTSTCT1; reg[22].val = 0x0005BD; //1469
  reg[23].addr = AFE4404_ADCRSTENDCT1; reg[23].val = 0x0005C3; //1475

  //ADC reset phase 2 start and end
  reg[24].addr = AFE4404_ADCRSTSTCT2; reg[24].val = 0x0009E9; //2537
  reg[25].addr = AFE4404_ADCRSTENDCT2; reg[25].val = 0x0009EF; //2543

  //ADC reset phase 3 start and end
  reg[26].addr = AFE4404_ADCRSTSTCT3; reg[26].val = 0x000E15; // 3605
  reg[27].addr = AFE4404_ADCRSTENDCT3; reg[27].val = 0x000E1B; // 3611

  //LED 3 start and end
  reg[28].addr = AFE4404_LED3LEDSTC; reg[28].val = 0x000190; //400
  reg[29].addr = AFE4404_LED3LEDENDC; reg[29].val = 0x00031F; //799

//  reg[28].addr = AFE4404_LED3LEDSTC; reg[28].val = 0x000000; //400
//  reg[29].addr = AFE4404_LED3LEDENDC; reg[29].val = 0x000000; //799

  reg[30].addr = AFE4404_PRPCT; reg[30].val = 0x009C3F; // 39999 -----  PRF counter - 1D
  
  reg[31].addr = AFE4404_TIMER_EN; reg[31].val = 0x000103; //259
  
  reg[32].addr = AFE4404_ENSEP_GAIN; reg[32].val = 0x008003; // 32771
  
  reg[33].addr = AFE4404_PROG_TG_EN; reg[33].val = 0x000003; // 3 //AFE_TIA_GAIN (LED1) LED1/LED1AMB gain = 50K
  
  reg[34].addr = AFE4404_DAC_SETTING_REG; reg[34].val = 0x000000; //AFE4404_DAC_SETTING_REG

  //LED control - Dec 34 - 0x22
  reg[35].addr = AFE4404_REG_LED_CONFIGURATION; reg[35].val = 0x0030CF; // 12495 //LED3 - 3.125mA; LED2 - 3.125mA; LED1 - 12.5mA
//  reg[35].addr = AFE4404_REG_LED_CONFIGURATION; reg[35].val = 0x000005; // 12495 //LED3 - 0mA; LED2 - 0mA; LED1 - 8mA
//   reg[35].addr = AFE4404_REG_LED_CONFIGURATION; reg[35].val = 0x000400; //Only LED2
   
//  reg[36].addr = AFE4404_DYN1_LED_DYN2_OSC_DYN3_DYN4; reg[36].val = 0x124018; //1196056
  reg[36].addr = AFE4404_DYN1_LED_DYN2_OSC_DYN3_DYN4; reg[36].val = 0x124218;

  reg[37].addr = CLKDIV_PRF; reg[37].val = 0x000000; //0 //Clock Division setting
  
  reg[38].addr = AFE4404_PDNCYCLESTC; reg[38].val = 0x00155F; //5471
  
  reg[39].addr = AFE4404_PDNCYCLEENDC; reg[39].val = 0x00991F; // 39199
  

  for (i = 0; i < NUM_REGISTERS; i++)
    writeData(reg[i].addr, reg[i].val);

}

void AFE4404::initClock(void) {

//  LPC_PWM1->TCR = (1 << 1);               // Reset counter, disable PWM
//  LPC_SC->PCLKSEL0 &= ~(0x3 << 12);
//  LPC_SC->PCLKSEL0 |= (1 << 12);          // Set peripheral clock divider to /1, i.e. system clock
//  LPC_PWM1->MR0 = 22;                     // Match Register 0 is shared period counter for all PWM1
//  LPC_PWM1->MR6 = 11;                      // Pin 21 is PWM output 6, so Match Register 6
//  LPC_PWM1->LER |= 1;                     // Start updating at next period start
//  LPC_PWM1->TCR = (1 << 0) || (1 << 3);   // Enable counter and PWM
  _clk.period(0.01f);      // 4 second period
  _clk.write(0.50f); 

}


//
//void AFE4404::powerUpSequence(void) {
//
//  initPorts();
//  initPowerSupply();
//  initRegisters();
//  initClock();
////  _drdy.rise(this, &AFE4404::getData);
//  enableIRQ();
//
//}
//void AFE4404::getData(void) {
//
//  disableIRQ();
//  data = static_cast<int32_t> (readData(LED, true));
//  dataAvailable = true;
//  enableIRQ();
//}
//uint32_t AFE4404::readData(uint8_t reg, bool adc = true) {
//
//  if(!adc) {
//    enableReadMode();
//  }
//  
//  writeBuffer[0] = reg;
//
//
// readBuffer[0] = 0x00;
// readBuffer[1] = 0x00;
// readBuffer[2] = 0x00;
//
//  Wire.beginTransmission(AFE4404_I2C_ADDRESS);
//  Wire.write(writeBuffer[0]);
////  Wire.write(writeBuffer, 1);
//  
//  
////  Wire.write(AFE4404_I2C_ADDRESS);
////  Wire.endTransmission(false);
////   Wire.requestFrom((uint8_t)AFE4404_I2C_ADDRESS, 3);
//  Wire.read(readBuffer, 3);
//
//  tempData = 0;
//
//  tempData = (readBuffer[0] << (BITS_PER_BYTE * 2)) | (readBuffer[1] << BITS_PER_BYTE) | readBuffer[2];
//
//  return tempData;
//
//}
//void AFE4404::writeData(uint8_t reg, uint32_t data) {
//
//  enableWriteMode();
//  writeBuffer[0] = reg;
//
//  // store the lower 3 bytes of data in writeBuffer (MSB first)
//  for (int i=2, j = 1; i >= 0; i--, j++) {
//    writeBuffer[j] = (data >> (BITS_PER_BYTE * i)) & LOWER_BYTE_MASK;
//  }
//  //write 4 bytes
//  // 1 : register address and 3 for lower 3 bytes of data
//  Wire.beginTransmission(AFE4404_I2C_ADDRESS);
//  Wire.write(writeBuffer[0]); //reg
//  Wire.write(writeBuffer[1]); //MSB
//  Wire.write(writeBuffer[2]);
//  Wire.write(writeBuffer[3]);
//  Wire.endTransmission();
//  
//  
//}
