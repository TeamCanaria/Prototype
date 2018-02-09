#include "i2c_master.h"
/*
Writes setBit to targetBit in targetRegister on targetDeviceRead/Write.
Reads targetRegister, changes desired bit and writes back.
*/
void i2c_write_single(uint8_t targetDeviceWrite, uint8_t targetDeviceRead,
		uint8_t targetRegister, uint8_t targetBit, uint8_t setBit) {
	
	uint8_t inputRaw = 0x00;
	
	i2c_start(targetDeviceWrite);
	i2c_write(targetRegister);
	i2c_stop();
	
	i2c_start(targetDeviceRead);
	inputRaw = i2c_read_nack();
	i2c_stop();
	
	if (setBit) { // Set bit to 1.
		inputRaw |= (1 << targetBit);
	} else { // Set bit to 0.
		inputRaw &= ~(1 << targetBit);
	}
	
	i2c_start(targetDeviceWrite);
	i2c_write(targetRegister);
	i2c_write(inputRaw);
	i2c_stop();
}

/*
Initialise the AFE.
*/
void initialise_AFE(void) {
	// uint8_t outputRaw = 0x00;
}

/*
Reading the AFE.
*/
void read_AFE(void) {
	// uint8_t inputRaw = 0x00;
}

void enable_PC_ISR(void) {
	PCICR |= (1 << PCIE2); // Pin Change Interrupt Enable 2 (23:16)
	PCMSK2 |= (1 << PCINT23); // Pin CHange Mask Register to enable PCINT23.
}

/* Handles Pin Change Interrupt Request */
ISR(PCINT2_vect) {
	// Read from AFE.
	return;
}