#ifndef USART_H_
#define USART_H_

#define STRINGSIZE 10
#define FOSC 8000000		// Clock speed
#define BAUD 250000
#define SETUBRR FOSC/16/BAUD-1

/*
	Initialise UART communication on Rx/Tx pins.
*/
void initialise_USART(void);

/*
	Sends an 8-bit value to UDR0 for transmission via Tx.
*/
void send_data(unsigned char data);

/*
	Calls send_data() for a given string.
*/
void send_message(char* output);

/*
	Sends a 16-bit value as binary.  Use to send ADC average results in binary.
*/
void send_result_as_binary(uint16_t output);

/*
	Sends a 16-bit integer as a string of characters.  Use to send ADC average
	results in ASCII.
*/
void send_result_as_ascii(uint16_t output);

/*
	Services Data Reception from USART.  Packet received determines the
	function call.
*/
void handle_rx(void);

/*
	Handles USART Receive Interrupt Request
	rxFlag is serviced later in main()s Functioning loop.
*/
ISR(USART_RX_vect);


#endif /* USART_H_ */