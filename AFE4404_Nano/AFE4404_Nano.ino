#ifdef ARDUINO
#include <Wire.h>
#endif

#include "AFE4404.h"
//#include "i2c_t3.h"
#include "mbed.h"

//#include <Wire.h>

//// DIGITAL OUTPUTs
//#define PIN_RX    23
//#define PIN_TX    22
//#define PIN_PWR   21
//#define PIN_RST   17
//
//#define PIN_DRDY  15
//#define PIN_CLK   20
//#define PIN_SDA   18
//#define PIN_SCL   19


// DIGITAL OUTPUTs
//#define PIN_TX    D0
//#define PIN_RX    D1
//#define PIN_PWR   D2
#define PIN_RST   D3

#define PIN_DRDY  D2
#define PIN_CLK   D5
#define I2C_SDA   D18 
#define I2C_SCL   D19
AFE4404 afe(PIN_RST, PIN_DRDY, PIN_CLK , I2C_SDA, I2C_SCL);

//void initPorts(void) {
//  pinMode(PIN_RX, OUTPUT);
//  digitalWrite(PIN_RX, LOW);
//
//  pinMode(PIN_TX, OUTPUT);
//  digitalWrite(PIN_TX, LOW);
//
//  pinMode(PIN_PWR, OUTPUT);
//  digitalWrite(PIN_PWR, LOW);
//
//  pinMode(PIN_RST, OUTPUT);
//  digitalWrite(PIN_RST, HIGH);
//  
//}

RawSerial pc(USBTX, USBRX);
extern int32_t data;
extern bool dataAvailable;
volatile bool isAdcReady = false;
long timeStamp = millis();
char LED = 0x2A;
void powerUpSequence(void);



void setup()
{
  pc.printf("Setting up....\n");
  pc.baud(115200);
  
//  pinMode(PIN_DRDY, INPUT_PULLUP);
  
  powerUpSequence();
  interruptSetup();
}

void getData(void) {

  afe.disableIRQ();
  data = static_cast<int32_t> (afe.readData(LED, true));
//  pc.printf("%d ", data);
  dataAvailable = true;
  afe.enableIRQ();
}


void powerUpSequence(void) {

//  afe.initPorts();
////  afe. initPowerSupply();
//  afe.initRegisters();
////  afe.initClock();
////  _drdy.rise(this, &AFE4404::getData);
//  afe._drdy.rise(&getData);
//  afe.enableIRQ();



  //***********
//  pinMode(PIN_RST, OUTPUT);
//  digitalWrite(PIN_RST, HIGH);
    afe.writeData(0x00, 0x08);
//  Wire.begin(4, 5);
  //  Wire.begin();
  delay(500);
  pc.printf( "TWI Initialized\n" );
  //Toggle Reset pin
  digitalWrite(PIN_RST, LOW);
  delayMicroseconds(50);
  digitalWrite(PIN_RST, HIGH);

  //Heart rate 3 init
  
  afe.initRegisters();
  pc.printf( "HR3 Initialized" );
  delay(100);

}
void test() {
  for (uint32_t reg = 0x01; reg < 0x40 ;reg++) {  
    long a; 
    afe.writeData(reg,0xAAAAAA); 

    afe.disableIRQ();  
    a = afe.readData(reg, true);
    afe.enableIRQ(); 

    if (a != 0xAAAAAA) {  
      pc.printf("Register %X mistmatch\n",reg);
      pc.printf("Read Data Val: %X\n", a);
    } else {
      pc.printf("Reg %X match\n", reg);
    }
  }


}
void loop() {
  // put your main code here, to run repeatedly:
//  getData();
//  if (dataAvailable) {
//    
//    //data = afe.readData(AFE4404_LED2VAL, true);
//    pc.printf("%d\n", data);
//    dataAvailable = false;
//  }
//
//  pc.printf("Measuring...\n");
//  pc.printf("Interupt Pin :%d\n", PIN_DRDY);
//  pc.printf("SDA: %d and SCL: %d\n", D18, D19);
  if(isAdcReady){
//      test();
     
      data = afe.readData(LED, true);
      pc.printf("%X\n", data);
      timedEvents();
      isAdcReady = false;
  }
  
  
}
void timedEvents(){
  if(millis() - timeStamp > 200){
//    sendRate();
    interrupts();    
    timeStamp = millis();
  }  
}


void interruptSetup( void ){
  attachInterrupt(digitalPinToInterrupt(PIN_DRDY), adc_rdy_isr, RISING);
}

void adc_rdy_isr(){
  
  pc.printf("INTERRUPT\n");
  isAdcReady = true;
}

