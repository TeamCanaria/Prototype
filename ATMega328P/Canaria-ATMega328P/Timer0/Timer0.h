#ifndef TC0_H_
#define TC0_H_

/*
	Used to extend the upper limit of the timestamp.  Timestamp collected from
	TCNT0 (8-bit register).  Overflow increments timeStampMulti.
	Maximum timestamp is now (2^8) * (2^32) ~= 10 ^ 12 us (305 Hours)
*/
uint32_t timeStampMulti;

/*
	Resets relative time stamp.
*/
void set_time_stamp(void);

/*
	Handles Timer/Counter 0 Overflow Interrupt Request
	Called when TCNT0 register overflows.  Increments 32-bit timeStampMulti to
	extend timestamp to 40 bits.
*/
ISR(TIMER0_OVF_vect);

/*
Initialise TC0 registers for 'Normal' operation.
*/
void initialise_TC0(void);



#endif /* TC0_H_ */