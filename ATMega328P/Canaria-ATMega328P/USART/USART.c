#ifndef MAIN_H_
#include "../main.h"
#endif /* MAIN_H_ */

#ifndef USART_H_
#include "USART.h"
#endif /* USART_H_ */

#ifndef TIMER_0_
#include "../Timer0/Timer0.h"
#endif /* TIMER_0_ */

#ifndef TIMER_1_
#include "../Timer1/Timer1.h"
#endif /* TIMER_1_ */

#include <string.h> // strlen()
#include <stdlib.h> // itoa()

/*
	Sends an 8-bit value to UDR0 for transmission via Tx.
*/
void send_data(unsigned char data) {
	/*
		Waits for UDR0 to be cleared (by receiving or transmitting).
	*/
	while (!((1 << UDRE0) & UCSR0A));
	/*
		USART Data Register filled with data.  Hardware logic handles
		transmission and timing.
	*/
	UDR0 = data;
}

/*
	Calls send_data() for a given string.
*/
void send_message(char* output) {
	int length = strlen(output);

	/*
		Sends string upto and excluding the first null character ('\0').
	*/
	for (int i = 0; i < length; i++) {
		send_data(output[i]);
	}
}

/*
	Sends a 16-bit value as binary.  Use to send ADC average results in binary.
*/
void send_result_as_binary(uint16_t output) {
	/*
		Sends least-significant 8-bits.
	*/
	uint8_t rawOutput = output;
	send_data(rawOutput);

	/*
		Sends most-significant 8-bits.
	*/
	rawOutput = (output >> 8);
	send_data(rawOutput);
}

/*
	Sends a 16-bit integer as a string of characters.  Use to send ADC average
	results in ASCII.
*/
/*
void send_result_as_ascii(uint16_t output) {
	char[10] charOutput;
	memset(charOutput, '\0', STRINGSIZE);

	itoa(output, charOutput, 10);
	send_message(charOutput);
}
*/

/*
	Handles USART Receive Interrupt Request
	rxFlag is serviced later in main()s Functioning loop.
*/
ISR(USART_RX_vect) {
	/*
		Reads the USART Data Register into rxFlag.
	*/
	rxFlag = UDR0;
	return;
}

/*
	Services Data Reception from USART.  Packet received determines the
	function call.
*/
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

/*
	Initialise UART communication on Rx/Tx pins.
*/
void initialise_USART(void) {
	/*
		Enable USART from Power Management.
	*/
	PRR &= ~(1 << PRUSART0);
	
	/*
		Set baud rate from calculated UBRR.
	*/
	UBRR0H = (unsigned char)((SETUBRR) >> 8);
	UBRR0L = (unsigned char)(SETUBRR);
	
	/*
		Enable Double Speed mode.
	*/
	// UCSRA = (1 << U2X);
	
	/*
		Enable receiver and transmitter.
		Interrupt enabled on reception.  
	*/
	UCSR0B = ((1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0)) & ~((1 << UDRIE0) | (1 << UCSZ02));
	
	/*
		Asynchronous mode
		Packet Structure: 8 data bits, 1 stop bit, odd parity
	*/
	UCSR0C = ((1 << UCSZ01) | (1 << UCSZ00) | (1 << UPM01) | (1 << UPM00)) &
	~((1 << UMSEL01) | (1 << UMSEL00) | (1 << UCPOL0) | (1 << USBS0));
	
	return;
}
