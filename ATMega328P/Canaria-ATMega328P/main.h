#ifndef MAIN_H_
#define MAIN_H_

#define F_CPU 8000000UL
#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define Dev24C02  0xA2      // Device address of EEPROM 24C02, see datasheet

#define SAMPLERATE 100
#define METHODSTEPS 40
#define DEBUG 1

typedef enum {
	ONRED = 0,
	READRED = 1,
	OFFRED = 4,
	READAMB1 = 5,
	ONIR = 8,
	READIR = 9,
	OFFIR = 12,
	READAMB2 = 13,
	ONGREEN = 16,
	READGREEN = 17,
	OFFGREEN = 20,
	SENDRED = 21,
	SENDAMB1 = 23,
	SENDIR = 25,
	SENDAMB2 = 27,
	SENDGREEN = 29,
	RESET = (METHODSTEPS - 1)
} MethodFlag;

typedef enum {
	DOWN = 0,
	SETTIMESTAMP = 1,
	STARTSAMPLING = 2,
	STOPSAMPLING = 3
} RxFlag;

typedef struct {
	uint16_t red;
	uint16_t amb1;
	uint16_t ir;
	uint16_t amb2;
	uint16_t green;
} AverageStruct;

/* Global variables */
MethodFlag methodFlag;
RxFlag rxFlag;

#endif /* MAIN_H_ */