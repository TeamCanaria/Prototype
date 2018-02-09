#ifndef MAIN_H_
#include "../main.h"
#endif /* MAIN_H_ */

#ifndef TIMER0_H_
#include "Timer0.h"
#endif /* TIMER0_H_ */

/*
	Resets relative time stamp.
*/
void set_time_stamp(void) {
	/*
		Clear Timer/Counter 0 and Reset timeStampMulti.
	*/
	TCNT0 = 0x00;
	timeStampMulti = 0x00000000;
}

/*
	Handles Timer/Counter 0 Overflow Interrupt Request
	Called when TCNT0 register overflows.  Increments 32-bit timeStampMulti to
	extend timestamp to 40 bits.
*/
ISR(TIMER0_OVF_vect) {
	timeStampMulti += 1;
}

/*
Initialise TC0 registers for 'Normal' operation.
*/
void initialise_TC0(void) {
	/*
		Wave Generation Mode (WGM) Mode 0 - 'Normal'
		COM0A set to normal port operation (OC0A disconnected - 0b00)
		COM0B set to normal port operation (OC0B disconnected - 0b00)
		Clock Select (CS0[2:0]) set to start (0b010) with /8 prescaler.
	*/
	TCCR0A &= ~((1 << COM0A1) | (1 << COM0A0) | (1 << COM0B1) | (1 << COM0B0) |
			(1 << WGM01) | (1 << WGM00));
	TCCR0B |= (1 << CS01) & ~((1 << CS02) | (1 << CS00) | (1 << WGM02) |
			(1 << FOC0A) | (1 << FOC0B));

	/*
		Timer/Counter0 Overflow				TIMER0_OVF_vect
	*/
}