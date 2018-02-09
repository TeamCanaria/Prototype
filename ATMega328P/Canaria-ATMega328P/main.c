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
	//-ADC in Free-Running mode.// Removed.  Too slow waiting for ADC outside
			of ISR.  Reverted to on-demand conversions.
	- Timer0 to work as a timestamp.
	- Tx USART to send timestamp.
	- Rx USART responding to start/stop sampling and set timestamp.
*/

#include "main.h"
#include "Timer0/Timer0.h"
#include "Timer1/Timer1.h"
#include "USART/USART.h"
#include "ADC/ADC.h"
#include "AFE4404/AFE4404.h"

/*
	Initialise GPIO Pins.
*/
void initialise_GPIO(void) {
	/*
		PORTD6, PORTD7 and PORTB0 set as outputs.
	*/
	DDRD |= (1 << PORTD6) | (1 << PORTD7);
	DDRB |= (1 << PORTB0);
	
	/*
		PORTD6, PORTD7 and PORTB0 Initialised to logic LOW.
	*/
	PORTD &= ~((1 << PORTD6) | (1 << PORTD7));
	PORTB &= ~(1 << PORTB0);
	
	if (DEBUG) {
		/*
			DEBUG pin set to PORTC0.  Initialised to logic LOW.
		*/
		DDRB |= (1 << PORTC0);
		PORTC &= ~(1 << PORTC0);
	}
}

/*
	Calls initialisation functions and sets global variables.
*/
void initialisations(void) {
	/*
		Initialise registers grouped in functional blocks.
	*/
	initialise_USART();
	initialise_GPIO();
	initialise_TC0();
	initialise_TC1();
	initialise_ADC();
	
	/*
		Enable global interrupts.
	*/
	sei();
	
	/*
		Initialise Global Variables.
	*/
	isrService = 0x00;
	timeStampMulti = 0x00000000;
	
	return;
}


int main(void) {

	AverageStruct led;

	initialisations();

	start_sampling();

	/*
		Functioning loop.
	*/
	while(1) {

		if (isrService & (1 << TC1ISR)) {
			handle_tc1(&led);
			isrService &= ~(1 << TC1ISR);
		}

		if (rxFlag) {
			handle_rx();
			rxFlag = DOWN;
		}
	}
	return 0;
}

/*

Timer/Counter 1 to send time stamp on first led on.

Timer/Counter 1 from Mode 9 to Mode 4.

*/
