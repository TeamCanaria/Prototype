#ifndef MAIN_H_
#include "../main.h"
#endif /* MAIN_H_ */

#ifndef ADC_H_
	#include "ADC.h"
#endif /* ADC_H_ */
/*
	Initialise ADC registers for operation on ADCCHANNEL.
*/
void initialise_ADC(void) {
	PRR &= ~(1 << PRADC);
	
	/*
		Set channel of ADC.
	*/
	uint8_t channelADC = ADCCHANNEL & 0x07;
	ADMUX |= (channelADC);

	/*
		Reference ADC from GND to AVCC
	*/
	ADMUX |= (1 << REFS0) & ~(1 << REFS1);
	
	/*
		Auto Trigger Disabled
		ADC Interrupt Flag Cleared
		ADC Interrupt Disabled
	*/
	ADCSRA &= ~((1 << ADATE) | (1 << ADIF) | (1 << ADIE));
	
	/*
		ADC Prescaler "/2".  (8MHz to 4MHz)
	*/
	ADCSRA &= ~((1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0));
	return;
}

/*
	Enables ADC and starts a conversion.
*/
void start_adc(void) {
	/*
		Set:
			ADEN (ADC Enable)
			ADSC (ADC Start Conversion)
	*/
	ADCSRA |= (1 << ADEN) | (1 << ADSC);

	if (DEBUG) {
		PORTC |= (1 << PORTC0);
	}
}

/*
	Disables ADC (and ceases any current conversion).
*/
void stop_adc(void) {
	/*
		Clear:
			ADEN (ADC Enable)
			ADSC (ADC Start Conversion)
	*/
	ADCSRA &= ~((1 << ADEN) | (1 << ADSC));

	if (DEBUG) {
		PORTC &= ~(1 << PORTC0);
	}
}

/*
	Takes a set number of ADC readings (AVERAGES) and returns their sum.
*/
uint16_t sample_adc(void) {
	uint16_t result = 0x0000;
	for (uint8_t i = 0; i < AVERAGES; i++) {
		start_adc();
		while (ADCSRA & (1 << ADSC));
		result += ADC;
	}
	stop_adc();
	return result;
}