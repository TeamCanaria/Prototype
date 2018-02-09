/*
 *  Canaria-ATMega328P.c
 *
 *	Copyright (C) 2017  CanariaAlbert Dillon, Daniel Stojcevski, Beichen Xia
 *
 *	This program is free software: you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation, either version 3 of the License, or
 *	(at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *	CSSE2310 Style Guide 2.0.4
 *	
 *	Credits to GitHub's cpldcpu and g4lvanix for ws2812_config.h and
 *	i2c_master.h respectively.
 */

#define F_CPU 8000000UL
#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <string.h>
#include "i2c_master.h"

#define Dev24C02  0xA2      // Device address of EEPROM 24C02, see datasheet

#define TOTALLEDS 30
#define OUTERDISPLAY 12
#define FOSC 8000000		// Clock speed
#define BAUD 250000
#define MYUBRR FOSC/16/BAUD-1
#define ADCCHANNEL 0		// Also change ISR.
#define AVERAGES 32
#define DEBUG 1

typedef enum {
	IRADC1 = 0,
	IRADC2 = 1,
	RADC1 = 2,
	RADC2 = 3,
	AMBADC1 = 4,
	AMBADC2 = 5,
	DOWN = 7
} BTTxFlag;

typedef enum {
	ONRED = 0,
	READRED = 1,
	OFFRED = 2,
	READAMB = 3,
	ONIR = 4,
	READIR = 5,
	OFFIR = 6,
	SENDRESULT = 7,
	RESET = 20
} SensorFlag;

/* Global variables */
BTTxFlag btTxFlag;		/* BT Flag for transmitting multiple packets */
SensorFlag sensorFlag;
uint16_t tcScaler;
// uint16_t redResult, ambResult, irResult;
uint8_t adcCounter = AVERAGES;
uint32_t averageResult;


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
Sends 8 bits to Tx pin.
*/
void send_data(unsigned char data) {
	while (!((1 << UDRE0) & UCSR0A));
	UDR0 = data;
}

/* Handles USART Receive Interrupt Request */
ISR(USART_RX_vect) {
	// uint8_t rawInput = UDR0; // Read from buffer.
	return;
}

/* Handles Pin Change Interrupt Request */
ISR(PCINT2_vect) {
	return;
}

void enable_PC_ISR(void) {
	PCICR |= (1 << PCIE2); // Pin Change Interrupt Enable 2 (23:16)
	PCMSK2 |= (1 << PCINT23); // Pin CHange Mask Register to enable PCINT23.
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

void send_message(char* output) {
	int length = strlen(output);

	for (int i = 0; i < length; i++) {
		send_data(output[i]);
	}
}

void send_result(uint16_t output) {
	uint8_t rawOutput = output;
	send_data(rawOutput);
	rawOutput = (output >> 8);
	send_data(rawOutput);
}

/* Handles ADC Interrupt Request */
ISR(ADC_vect) {
	/*
	if ((sensorFlag - 1) == READRED) {
		redResult = 0x0000;
		redResult = ADC;
	} else if ((sensorFlag - 1) == READIR) {
		irResult = 0x0000;
		irResult = ADC;
	} else if ((sensorFlag - 1) == READAMB) {
		ambResult = 0x0000;
		ambResult = ADC;
	}*/
	uint16_t convResult = ADC;
	averageResult += convResult;
	return;
}

/* Called to start sampling */
void start_sampling(uint16_t freq) {
	
	tcScaler = 8;
	
	/* Setting TOP.  High byte must be written first */
	uint16_t top = 8000000 / (2 * tcScaler * (freq * 20));
	OCR1AH = (unsigned char)(top >> 8);
	OCR1AL = (unsigned char)(top);
	/* Set Timer Counter to 0 */
	TCNT1H = 0x00;
	TCNT1L = 0x00;
	
	/*
		Select a pre scaler
		0 1 /1
		010 /8 <--
		011 /64
		100 /256
		101 /1024
	*/
	TCCR1B |= (1 << CS11) & ~((1 << CS12) | (1 << CS10)); // Start Timer/Counter
}

/*
Handles ISR when TCNT1 == OCR1A.
*/
ISR(TIMER1_COMPA_vect) {
	if (sensorFlag == ONRED) {
		PORTD |= (1 << PORTD2);
	} else if (sensorFlag == OFFRED) {
		PORTD &= ~(1 << PORTD2);
	} else if (sensorFlag == ONIR) {
		PORTD |= (1 << PORTD4);
	} else if (sensorFlag == OFFIR) {
		PORTD &= ~(1 << PORTD4);
	} else if ((sensorFlag == READRED) || (sensorFlag == READAMB) || (sensorFlag == READIR)) {
		PORTD |= (1 << PORTD3);
		for (uint8_t i = 0; i < AVERAGES; i++) {
			ADCSRA |= (1 << ADSC); // Start Conversion.
			while ((1 << ADSC) & ADCSRA);
			averageResult += ADC;
		}
		// adcCounter = 0;
		PORTD &= ~(1 << PORTD3);
	} else if (sensorFlag == RESET) {
		sensorFlag = 0;
		return;
	}
	/*
	else if (sensorFlag == SENDRESULT) {
		send_result(redResult);
	} else if ((sensorFlag - 5) == SENDRESULT) {
		send_result(ambResult);
	} else if ((sensorFlag - 10) == SENDRESULT) {
		send_result(irResult);
	}
	*/
	sensorFlag += 1;
	return;
}

/*
Stop Timer-Counter1
*/
void stop_sample(void) {
	// Turn off Counters PRR
	PRR |= (1 << PRTIM2) | (1 << PRTIM1) | (1 << PRTIM0) | (1 << PRADC);
	TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));
}

/*
Initialise PWM registers for 'phase-correct PWM' on Port X, Pin n.
*/
void initialise_TC1(void) {
	
	/*
	Wave Generation Mode (WGM) Mode 9 - 'Phase and Frequency Correct PWM'
	COM1A set to clear on up-counting, set on down-counting (0b10)
	COM1B set to Normal Port Operation (OC1B disconnected (0b00)
	Clock Select (CS1[2:0]) set to stop (0b000)
	*/
	DDRD |= (1 << PORTD2) | (1 << PORTD4);
	PORTD &= ~((1 << PORTD2) | (1 << PORTD4));

	if (DEBUG) {
		DDRD |= (1 << PORTD3);
		PORTD &= ~(1 << PORTD3);
	}
	
	TCCR1A |= ((1 << WGM10) | (1 << COM1A0)) & ~((1 << COM1A1) |
			(1 << COM1B1) | (1 << COM1B0) | (1 << WGM11));
	TCCR1B |= (1 << WGM13) & ~((1 << WGM12) | (1 << CS12) | (1 << CS11) |
			(1 << CS10));
	
	/* Timer/Counter1 Compare Match A	TIMER1_COMPA_vect */
	TIMSK1 |= (1 << OCIE1A);
	
	return;
}

void initialise_ADC(uint8_t channelADC) {
	PRR &= ~(1 << PRADC);
	
	channelADC &= 0x07;  // Sets binary of selected ADC port.
	
	ADMUX |= (1 << REFS0) & ~(1 << REFS1); // Reference to AVCC
	ADMUX |= (channelADC); // Set Channel.
	
	/*
	ADC Enabled.
	ADC Interrupt Flag Set
	ADC Interrupt Enabled
	*/
	ADCSRA = ((1 << ADEN) | (1 << ADIF)); // | (1 << ADATE) | (1 << ADIF) | (1 << ADIE));
	
	/*
	ADC Prescaler "/2".  (8MHz to 4MHz)
	*/
	ADCSRA &= ~((1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0));
	return;
}

/*
Takes a 16-bit UBRR and sets baud rate.
Enables USART communication.
*/
void initialise_USART(uint16_t ubrr) {
	PRR &= ~(1 << PRUSART0);
	
	// Set baud rate
	UBRR0H = (unsigned char)(ubrr >> 8);
	UBRR0L = (unsigned char)ubrr;
	
	// Enable Double Speed mode. Not required.
	/* UCSRA = (1 << U2X); */
	
	/* Enable receiver and transmitter. */
	UCSR0B = ((1 << TXEN0)) & ~((1 << UDRIE0) | (1 << UCSZ02)); // (1 << RXEN0) | (1 << RXCIE0)
	
	/* Asynchronous mode, 8N1 */
	UCSR0C = ((1 << UCSZ01) | (1 << UCSZ00)) &
	~((1 << UMSEL01) | (1 << UMSEL00) | (1 << UCPOL0) | (1 << UPM00) | (1 << USBS0) | (1 << UPM01));
	
	return;
}

/*
Calls initialisation functions and sets global variables.
*/
void initialisations(void) {
	/* Initialise registers of the ATMega328P */
	initialise_USART(MYUBRR); /* Calculated UBRR = 15 for 115200 baud */
	initialise_TC1();
	initialise_ADC(ADCCHANNEL);
	sei(); // Enable global interrupts.
	
	/* Initialise Global Variables */
	btTxFlag = DOWN;
	
	return;
}

int main(void) {
	initialisations(); // Sets registers for operation.
	start_sampling(100);
	while(1) {
		/*
		if (adcCounter < AVERAGES) {
			ADCSRA |= (1 << ADSC); // Start Conversion.
			adcCounter += 1;
			while ((1 << ADSC) & ADCSRA);
		}
		if (1 || (adcCounter >= AVERAGES)) {
			PORTD &= ~(1 << PORTD3);
		}
		*/
		_delay_ms(1000);
	}
	return 0;
}

/*
Timer Counter ISR:
Start first conversion.
Reset global counter.
DEBUG: Signal a GPIO to low.

ADC ISR:
Add ADC value to global averages.
Increment global counter.
Compare to 16/32/64.
DEBUG: When compared, signal a GPIO pin to high.

*/