

#ifndef AFE4404_REGISTERS_H
#define AFE4404_REGISTERS_H

#define AFE4404_I2C_ADDRESS                    0x58 << 1

// Interrupt enable register
#define AFE4404_LED2STC                         0x01
#define AFE4404_LED2ENDC                        0x02
#define AFE4404_LED1LEDSTC                      0x03
#define AFE4404_LED1LEDENDC                     0x04
#define AFE4404_ALED2STC_LED3STC                0x05
#define AFE4404_ALED2ENDC_LED3ENDC              0x06
#define AFE4404_LED1STC                         0x07
#define AFE4404_LED1ENDC                        0x08
#define AFE4404_LED2LEDSTC                      0x09
#define AFE4404_LED2LEDENDC                     0x0A
#define AFE4404_ALED1STC                        0x0B
#define AFE4404_ALED1ENDC                       0x0C
#define AFE4404_LED2CONVST                      0x0D
#define AFE4404_LED2CONVEND                     0x0E
#define AFE4404_ALED2CONVST_LED3CONVST          0x0F
#define AFE4404_ALED2CONVEND_LED3CONVEND        0x10
#define AFE4404_LED1CONVST                      0x11
#define AFE4404_LED1CONVEND                     0x12
#define AFE4404_ALED1CONVST                     0x13
#define AFE4404_ALED1CONVEND                    0x14
#define AFE4404_ADCRSTSTCT0                     0x15
#define AFE4404_ADCRSTENDCT0                    0x16
#define AFE4404_ADCRSTSTCT1                     0x17
#define AFE4404_ADCRSTENDCT1                    0x18
#define AFE4404_ADCRSTSTCT2                     0x19
#define AFE4404_ADCRSTENDCT2                    0x1A
#define AFE4404_ADCRSTSTCT3                     0x1B
#define AFE4404_ADCRSTENDCT3                    0x1C
#define AFE4404_PRPCT                           0x1D

 
#define AFE4404_TIMER_EN                        0x1E
#define AFE4404_ENSEP_GAIN                      0x20
#define AFE4404_PROG_TG_EN                      0x21
#define AFE4404_DYN1_LED_DYN2_OSC_DYN3_DYN4     0x23


// These registers (2Ah-2Fh) below are for READ mode only
#define AFE4404_LED2VAL                         0x2A  
#define AFE4404_ALED2VAL_LED3VAL                0x2B
#define AFE4404_LED1VAL                         0x2C
#define AFE4404_ALED1VAL                        0x2D
#define AFE4404_LED2_ALED2VAL                   0x2E
#define AFE4404_LED1_ALED1VAL                   0x2F

#define AFE4404_PDNCYCLESTC                     0x32
#define AFE4404_PDNCYCLEENDC                    0x33
#define AFE4404_PROG_TG_STC                     0x34
#define AFE4404_PROG_TG_ENDC                    0x35
#define AFE4404_LED3LEDSTC                      0x36
#define AFE4404_LED3LEDENDC                     0x37
#define CLKDIV_PRF                              0x39

#define AFE4404_DAC_SETTING_REG                 0x3A
#define AFE4404_AVG_LED2_ALED2VAL               0x3F
#define AFE4404_AVG_LED1_ALED1VAL               0x40


// LED Configuration register
#define AFE4404_REG_LED_CONFIGURATION          0x22
typedef enum LEDCurrent {
  AFE4404_LED_CURR_0MA      = 0x00,
  AFE4404_LED_CURR_0_8MA    = 0x01,
  AFE4404_LED_CURR_1_6MA    = 0x02,
  AFE4404_LED_CURR_2_4MA     = 0x03,

  AFE4404_LED_CURR_50MA     = 0x3f
} LEDCurrent;





#endif
