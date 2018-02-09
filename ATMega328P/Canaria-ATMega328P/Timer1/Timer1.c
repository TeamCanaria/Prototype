#ifndef MAIN_H_
#include "../main.h"
#endif /* MAIN_H_ */

#ifndef ADC_H_
#include "../ADC/ADC.h"
#endif /* ADC_H_ */

#ifndef TIMER1_H_
#include "Timer1.h"
#endif /* TIMER1_H_ */

#ifndef USART_H_
#include "../USART/USART.h"
#endif /* USART_H_ */

/*
	Starts Timer/Counter 1.  Initiates main method.
*/
void start_sampling(void) {
	
	/*
		Setting TOP of Timer/Counter 1.  High byte must be written first.
		Given a Sample Rate of 100Hz, methodSteps will further divide the
	*/
	uint16_t tcScaler = 8;
	uint16_t top = 8000000 / (2 * tcScaler * (SAMPLERATE * METHODSTEPS));
	OCR1AH = (unsigned char)(top >> 8);
	OCR1AL = (unsigned char)(top);
	
	/*
		Set Timer Counter to 0
	*/
	TCNT1H = 0x00;
	TCNT1L = 0x00;
	
	/*
		Selecting a Clock Source will initiate Timer/Counter.

		Select a pre scaler in TCCR1B.CS1[2:0]
		000		Disabled.
		0 1		/1
		010		/8 <--
		011		/64
		100		/256
		101		/1024
	*/
	TCCR1B |= (1 << CS11) & ~((1 << CS12) | (1 << CS10));
}

/*
	Stops Timer/Counter 1.  Halts main method.
*/
void stop_sampling(void) {
	/*
		Deselects a clock source and stops Timer/Counter 1.
	*/
	TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));

	stop_adc();

	/*
		Turns off Timer/Counter 1 and ADC in Power Management
	*/
	PRR |= (1 << PRTIM1) | (1 << PRADC);
	
	/*
		Resets current step in main method for next start.
	*/
	methodFlag = 0;
}

/*
	Services Timer/Counter 1 Comparison Interrupt Request (TCNT1 == OCR1A).
	Main method.
	Handles the timing of LEDs, ADC sampling and result streaming.
*/
void handle_tc1(AverageStruct* led) {
	if (methodFlag == READRED) {
		led->red = sample_adc();
		send_message("r");
		send_result_as_binary(led->red);
	} else if (methodFlag == READAMB1){
		led->amb1 = sample_adc();
		send_message("1");
		send_result_as_binary(led->amb1);
	} else if (methodFlag == READIR) {
		led->ir = sample_adc();
		send_message("i");
		send_result_as_binary(led->ir);
	} else if (methodFlag == READAMB2) {
		led->amb2 = sample_adc();
		send_message("2");
		send_result_as_binary(led->amb2);
	} else if (methodFlag == READGREEN) {
		led->green = sample_adc();
		send_message("g");
		send_result_as_binary(led->green);
	} else if (methodFlag == RESET) {
		methodFlag = 0;
		return;
	} else {
		/*
			Fail-safe if ADC is still reading at time of LED transition.
			Will produce a false-negative in stead of false-positive.
		*/
		stop_adc();

		if (methodFlag == ONRED) {
			/*************************** Send a Time Stamp ******************************/
			PORTD |= (1 << PORTD6);
		} else if (methodFlag == OFFRED) {
			PORTD &= ~(1 << PORTD6);
		} else if (methodFlag == ONIR) {
			PORTD |= (1 << PORTD7);
		} else if (methodFlag == OFFIR) {
			PORTD &= ~(1 << PORTD7);
		} else if (methodFlag == ONGREEN) {
			PORTB |= (1 << PORTB0);
		} else if (methodFlag == OFFGREEN) {
			PORTB &= ~(1 << PORTB0);
		}
	}

	methodFlag += 1;
	return;
}

/*
	Flags main()s functioning loop to call handle_tc1().
*/
ISR(TIMER1_COMPA_vect) {
	isrService |= (1 << TC1ISR);
	return;
}

/*
	Initialise TC1 registers for 'phase-correct PWM'.
*/
void initialise_TC1(void) {
	/*
		Wave Generation Mode (WGM) Mode 4 - 'CTC' - 0b0100
		COM1A set to Normal Port Operation (OC1A disconnected - 0b00)
		COM1B set to Normal Port Operation (OC1B disconnected - 0b00)
		Clock Select (CS[2:0]) set to stop (0b000)
	*/
	TCCR1A &= ~((1 << WGM11) | (1 << WGM10) | (1 << COM1A1) | (1 << COM1A0) |
			(1 << COM1B1) | (1 << COM1B0));
	TCCR1B |= (1 << WGM12) & ~((1 << WGM13) | (1 << CS12) | (1 << CS11) | (1 << CS10));
	/*
		Timer/Counter1 Compare Match A		TIMER1_COMPA_vect
	*/
	TIMSK1 |= (1 << OCIE1A);
	
	return;
}