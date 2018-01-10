#ifndef AFE4404_H
#define AFE4404_H

#include <stdint.h>
#include "AFE4404_Registers.h"
#include "mbed.h"


#define LOWER_BYTE_MASK     0x000000FF
#define SIGN_EXT            0xFF000000
#define SIGN_MASK           0x00800000
#define BITS_PER_BYTE       8
#define NUM_REGISTERS       40

class AFE4404 {
  public:
        InterruptIn _drdy;
//      AFE4404(PinName rxSupplyEn, PinName txSupplyEn, PinName resetz, 
//            PinName powerEn, PinName drdy, PinName clk, PinName sda, PinName scl);
      AFE4404(PinName resetz, PinName drdy, PinName clk, PinName sda, PinName scl);

      void initPorts(void);
      void initPowerSupply(void);
      void initRegisters(void);
      void initClock(void);
      void powerUpSequence(void);
      
      void inline enableWriteMode(void) {
        _writeBuffer[0] = 0x00;    // AFE register address 0x00
        // write 0 to REG_READ bit in register 0x00 to enable readout 
        // of write registers
        _writeBuffer[1] = 0x00;    
        _writeBuffer[2] = 0x00;
        _writeBuffer[3] = 0x00;
        _i2c.write(_address, _writeBuffer, 4);
        }
        
        void inline enableReadMode(void) {
            _writeBuffer[0] = 0x00;    // AFE register address 0x00
            // write 1 to REG_READ bit in register 0x00 to enable writes to 
            // write registers
            _writeBuffer[1] = 0x00;    
            _writeBuffer[2] = 0x00;
            _writeBuffer[3] = 0x01;
            _i2c.write(_address, _writeBuffer, 4);           
        }  
//      void inline enableWriteMode(void) {
//        writeBuffer[0] = 0x00;
//        writeBuffer[1] = 0x00;
//        writeBuffer[2] = 0x00;
//        writeBuffer[3] = 0x00;
//        Wire.beginTransmission(AFE4404_I2C_ADDRESS);
//        Wire.write(writeBuffer[0]); //reg
//        Wire.write(writeBuffer[1]); //MSB
//        Wire.write(writeBuffer[2]);
//        Wire.write(writeBuffer[3]);
//        Wire.endTransmission();
//        
//      }
//
//      void inline enableReadMode(void) {
//        writeBuffer[0] = 0x00;
//        writeBuffer[1] = 0x00;
//        writeBuffer[2] = 0x00;
//        writeBuffer[3] = 0x01;
//        Wire.beginTransmission(AFE4404_I2C_ADDRESS);
//        Wire.write(writeBuffer[0]); //reg
//        Wire.write(writeBuffer[1]); //MSB
//        Wire.write(writeBuffer[2]);
//        Wire.write(writeBuffer[3]);
//        Wire.endTransmission();
//        
//      }

    void inline disableIRQ(void) {
          _drdy.disable_irq();
      }

      void inline enableIRQ(void) {
          _drdy.enable_irq();
      }
//      void getData(void);
      void writeData(uint8_t reg, uint32_t data);
      uint32_t readData(uint8_t reg, bool adc);
      
   private:
//      DigitalOut  _rxSupplyEn;
//      DigitalOut  _txSupplyEn;
      DigitalOut  _resetz;
//      DigitalOut  _powerEn;
      
//      InterruptIn _drdy;
      PwmOut      _clk;
      I2C         _i2c;
   
      
      char _writeBuffer[5];
      char _readBuffer[5];
      int _address;
      uint32_t _tempData;
};

#endif

