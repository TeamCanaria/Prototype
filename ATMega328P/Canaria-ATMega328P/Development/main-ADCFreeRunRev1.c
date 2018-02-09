/*
 *  Canaria-ATMega328P.c
 *
 *	Copyright (C) 2018 Canaria
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
 *	Credits to GitHub's g4lvanix for i2c_master.h.
 */

/*
Development of:
	- ADC in Free-Running mode.
	- Timer0 to work as a timestamp.
	- Tx USART to send timestamp.
	- Rx USART responding to start/stop sampling and set timestamp.
*/

#define F_CPU 8000000UL
#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h> // itoa()
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h> // strlen()

#define Dev24C02  0xA2      // Device address of EEPROM 24C02, see datasheet

#define TOTALLEDS 30
#define OUTERDISPLAY 12
#define FOSC 8000000		// Clock speed
#define BAUD 250000
#define MYUBRR FOSC/16/BAUD-1
#define ADCCHANNEL 0
#define SAMPLERATE 100
#define AVERAGES 16
#define STRINGSIZE 10
#define DEBUG 1

#define TC1ISR 0
#define ADCISR 1

typedef enum {
	ONRED = 0,
	READRED = 1,
	OFFRED = 2,
	READAMB = 3,
	ONIR = 4,
	READIR = 5,
	OFFIR = 6,
	SENDRED = 7,
	SENDAMB = 10,
	SENDIR = 13,
	RESET = 20
} SensorFlag;

typedef enum {
	DOWN = 0,
	SETTIMESTAMP = 1,
	STARTSAMPLING = 2,
	STOPSAMPLING = 3
} RxFlag;

/* Global variables */
SensorFlag sensorFlag;
RxFlag rxFlag;
uint8_t adcCounter, isrService;
uint32_t redAverage, ambAverage, irAverage;
uint32_t timeStampMulti;

/*
Sends 8 bits to Tx pin.
*/
void send_data(unsigned char data) {
	while (!((1 << UDRE0) & UCSR0A));
	UDR0 = data;
}

void send_message(char* output) {
	int length = strlen(output);

	for (int i = 0; i < length; i++) {
		send_data(output[i]);
	}
}

/* Send binary of result */
void send_binary_as_binary(uint16_t) {
	uint8_t rawOutput = output;
	send_data(rawOutput);

	rawOutput = (output >> 8);
	send_data(rawOutput);
}

/* Send ASCII of result */
void send_binary_as_ascii(uint16_t output) {
	char[STRINGSIZE] charOutput;
	memset(charOutput, '\0', STRINGSIZE);

	itoa(output, charOutput, 10);
	send_message(charOutput);
}

/* Called to set time stamp */
void set_time_stamp(void) {
	// Start TC0.

	// Set TC0 counter to 0.

}

/* Called to start sampling */
void start_sampling(void) {
	
	uint16_t tcScaler = 8;

	/* Setting TOP.  High byte must be written first */
	uint16_t top = 8000000 / (2 * tcScaler * (SAMPLERATE * 20));
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
Stop Timer-Counter1
*/
void stop_sampling(void) {
	// Turn off TC1, ADC PRR
	PRR |= (1 << PRTIM1) | (1 << PRADC);
	TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));
	
	// Stop TC1

	
	// Reset Method.
	sensorFlag = 0;
}

void stop_adc(void) {
	ADCSRA &= ~((1 << ADEN) | (1 << ADSC));
}

/* Handles Timer/Counter 0 Overflow Interrupt Request*/
ISR(TIMER0_OVF_vect) {
	timeStampMulti += 1;
}

/* Handles USART Receive Interrupt Request */
ISR(USART_RX_vect) {
	rxFlag = UDR0; // Read from buffer.
	return;
}

/* Flags an ADC to read */
ISR(ADC_vect) {
	if ((sensorFlag - 1) == READRED) {
		redAverage += ADC;
	} else if ((sensorFlag - 1) == READIR) {
		irAverage += ADC;
	} else if ((sensorFlag - 1) == READAMB) {
		ambAverage += ADC;
	}
	isrService |= (1 << ADCISR);
	return;
}

/* Flags a step in method */
ISR(TIMER1_COMPA_vect) {
	isrService |= (1 << TC1ISR);
	return;
}

void handle_rx(void) {
	if (rxFlag == SETTIMESTAMP) {
		set_time_stamp();
	} else if (rxFlag == STARTSAMPLING) {
		start_sampling();
	} else if (rxFlag == STOPSAMPLING) {
		stop_sampling();
	}
	return;
}

/* Handles ADC Interrupt Request */
void handle_adc(void) {
	if (adcCounter >= AVERAGES - 1) { // Stop the ADC.
		stop_adc();
		if (DEBUG) {
			PORTD &= ~(1 << PORTD3);
		}
	}

	adcCounter += 1;

	isrService |= (1 << ADCISR);
	return;
}

/* Handles Timer/Counter 1 Comparison Interrupt Request (TCNT1 == OCR1A) */
void handle_tc1(void) {
	if (sensorFlag == ONRED) {
		PORTD |= (1 << PORTD2);
		redAverage = 0x00000000;
	} else if (sensorFlag == OFFRED) {
		stop_adc();
		PORTD &= ~(1 << PORTD2);
		ambAverage = 0x00000000;
	} else if (sensorFlag == ONIR) {
		stop_adc();
		PORTD |= (1 << PORTD4);
		irAverage = 0x00000000;
	} else if (sensorFlag == OFFIR) {
		stop_adc();
		PORTD &= ~(1 << PORTD4);
	} else if ((sensorFlag == READRED) || (sensorFlag == READAMB) || (sensorFlag == READIR)) {
		ADCSRA |= (1 << ADSC); // Start Conversion.
		adcCounter = 0;
		if (DEBUG) {
			PORTD |= (1 << PORTD3);
		}
	} else if (sensorFlag == RESET) {
		sensorFlag = 0;
		return;
	} else if (sensorFlag == SENDRED) {
		send_message("Red,");
		send_result(redAverage);
	} else if (sensorFlag == SENDAMB) {
		send_message("Amb: ");
		send_result(ambAverage);
	} else if (sensorFlag == SENDIR) {
		send_message("nIR: ")
		send_result(irAverage);
	}
	sensorFlag += 1;
	return;
}

/*
Initialise TC0 registers for '' on Port X, Pin n.
*/
void initialise_TC0(void) {

}

/*
Initialise TC1 registers for 'phase-correct PWM' on Port X, Pin n.
*/
void initialise_TC1(void) {
	
	/*
		PORTD2 and PORTD4 as outputs.  Initialised to logic LOW.
	*/
	DDRD |= (1 << PORTD2) | (1 << PORTD4);
	PORTD &= ~((1 << PORTD2) | (1 << PORTD4));

	if (DEBUG) {
		// DEBUG pin set to PORTD3.  Initialised to logic LOW.
		DDRD |= (1 << PORTD3);
		PORTD &= ~(1 << PORTD3);
	}
	
	/*
	Wave Generation Mode (WGM) Mode 9 - 'Phase and Frequency Correct PWM'
	COM1A set to clear on up-counting, set on down-counting (0b10)
	COM1B set to Normal Port Operation (OC1B disconnected (0b00)
	Clock Select (CS1[2:0]) set to stop (0b000)
	*/
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
	ADC Enabled
	Auto Trigger Enabled
	ADC Interrupt Flag Cleared
	ADC Interrupt Enabled
	*/
	ADCSRA = ((1 << ADEN) | (1 << ADATE) | (1 << ADIF) | (1 << ADIE));
	
	/*
	ADC Prescaler "/2".  (8MHz to 4MHz)
	*/
	ADCSRA &= ~((1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0));

	/*
	ADC to Free-Running Mode.
	*/
	ADCSRB &= ~((1 << ADTS2) | (1 << ADTS1) | (1 << ADTS0));
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
	UCSR0B = ((1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0)) & ~((1 << UDRIE0) | (1 << UCSZ02));
	
	/* Asynchronous mode, 8 data bits, 1 stop bit */
	UCSR0C = ((1 << UCSZ01) | (1 << UCSZ00) | (1 << UPM01) | (1 << UPM00)) &
	~((1 << UMSEL01) | (1 << UMSEL00) | (1 << UCPOL0) | (1 << USBS0));
	
	return;
}

/*
Calls initialisation functions and sets global variables.
*/
void initialisations(void) {
	/* Initialise registers of the ATMega328P */
	initialise_USART(MYUBRR); /* Calculated UBRR = 15 for 115200 baud */
	initialise_TC0();
	initialise_TC1();
	initialise_ADC(ADCCHANNEL);
	sei(); // Enable global interrupts.
	
	/* Initialise Global Variables */
	adcCounter = AVERAGES;
	isrService = 0x00;

	timeStampMulti = 0x00000000;
	
	return;
}

int main(void) {
	initialisations(); // Sets registers for operation.
	start_sampling();
	while(1) {
		if (isrService & (1 << TC1ISR)) {
			handle_tc1();
			isrService &= ~(1 << TC1ISR);
		}
		if (isrService & (1 << ADCISR)) {
			handle_adc();
			isrService &= ~(1 << ADCISR);
		}
		if (rxFlag) {
			handle_rx();
			rxFlag = DOWN;
		}
	}
	return 0;
}

/*
Prescale TC0 to 1 MHz (1us periods).

Tx to send Timer/Counter 0 value with data.

Increment 32-bit multiplier when overflow.

*/