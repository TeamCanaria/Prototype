  // #include "i2c_t3.h"
  // pinMode(PIN_DRDY, INPUT_PULLUP);
  // sendRate();
  
  /*
  getData();
  if (dataAvailable) {
    
    //data = afe.readData(AFE4404_LED2VAL, true);
    pc.printf("%d\n", data);
    dataAvailable = false;
  }

  pc.printf("Measuring...\n");
  pc.printf("Interupt Pin :%d\n", PIN_DRDY);
  pc.printf("SDA: %d and SCL: %d\n", D18, D19);
  */
  
    /*
  afe.initPorts();
  afe.initPowerSupply();
  afe.initRegisters();
  afe.initClock();
  _drdy.rise(this, &AFE4404::getData);
  afe._drdy.rise(&getData);
  afe.enableIRQ();
  */

  /*
  pinMode(PIN_RST, OUTPUT);
  digitalWrite(PIN_RST, HIGH);
  Wire.begin(4, 5);
  */
  
  /*
void initPorts(void) {
  pinMode(PIN_RX, OUTPUT);
  digitalWrite(PIN_RX, LOW);

  pinMode(PIN_TX, OUTPUT);
  digitalWrite(PIN_TX, LOW);

  pinMode(PIN_PWR, OUTPUT);
  digitalWrite(PIN_PWR, LOW);

  pinMode(PIN_RST, OUTPUT);
  digitalWrite(PIN_RST, HIGH);
  
}
*/

/*
void getData(void) {

  afe.disableIRQ();
  data = static_cast<int32_t> (afe.readData(LED, true));
  // pc.printf("%d ", data);
  dataAvailable = true;
  afe.enableIRQ();
}
*/

/*
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
*/

/*
#define PIN_RX    23
#define PIN_TX    22
#define PIN_PWR   21
#define PIN_RST   17

#define PIN_DRDY  15
#define PIN_CLK   20
#define PIN_SDA   18
#define PIN_SCL   19


// DIGITAL OUTPUTs
#define PIN_TX    D0
#define PIN_RX    D1
#define PIN_PWR   D2
*/