#ifndef TIMER1_H_
#define TIMER1_H_

/*
	Used to flag whether an ISR has been serviced.
	Bit 0: TC1ISR
		Bit is set when TIMER1_COMPA_vect is called.
		Bit is cleared after calling handle_tc1();
 */
#define TC1ISR 0
uint8_t isrService;

/*
	Starts Timer/Counter 1.  Initiates main method.
*/
void start_sampling(void);

/*
	Stops Timer/Counter 1.  Halts main method.
*/
void stop_sampling(void);

/*
	Services Timer/Counter 1 Comparison Interrupt Request (TCNT1 == OCR1A).
	Main method.
	Handles the timing of LEDs, ADC sampling and result streaming.
*/
void handle_tc1(AverageStruct* led);

/*
	Flags main()s functioning loop to call handle_tc1().
*/
ISR(TIMER1_COMPA_vect);

/*
	Initialise TC1 registers for 'phase-correct PWM'.
*/
void initialise_TC1(void);



#endif /* TIMER1_H_ */