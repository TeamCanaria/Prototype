#ifndef ADC_H_
#define ADC_H_

#define ADCCHANNEL 3
#define AVERAGES 128

/*
	Initialise ADC registers for operation on ADCCHANNEL.
*/
void initialise_ADC(void);

/*
	Enables ADC and starts a conversion.
*/
void start_adc(void);

/*
	Disables ADC (and ceases any current conversion).
*/
void stop_adc(void);

/*
	Takes a set number of ADC readings (AVERAGES) and returns their sum.
*/
uint16_t sample_adc(void);



#endif /* ADC_H_ */