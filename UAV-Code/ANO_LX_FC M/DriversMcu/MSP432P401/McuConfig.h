#ifndef _MCUCONFIG_H_
#define _MCUCONFIG_H_
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <time.h>
#include <stdlib.h>

typedef	uint32_t 	u32;
typedef	uint16_t 	u16;
typedef	uint8_t 	u8;
typedef	int32_t 	s32;
typedef	int16_t 	s16;
typedef	int8_t 		s8;

/*
引脚分配
串口		RX		TX
串口A0：	P1.2	P1.3
串口A1：	P2.2	P2.3
串口A2：	P3.2	P3.3
串口A3：	P9.6	P9.7
ADC：A0		P5.5
PWMOUT：	TA1：P7.4 P7.5 P7.6 P7.7
PPM：		TA2.1	P5.6
RGBO：		P8.3	P8.4	P8.5	P5.3
FLASHSPI：	UCB3：CLK：P10.1	SIMO：P10.2	SOMI：P10.3	CS：P10.0
SPI：		UCB0：CLK：P1.5		SIMO：P1.6	SOMI：P1.7	
CS1234：	P8.6	P8.7	P9.0	P9.1
备用PWM：	TA3：P10.4	P10.5	P8.2	P9.2

*/
#define INT_PRIORITY_UART		0x00
#define INT_PRIORITY_LXTIMER	0x40
#define INT_PRIORITY_ADC		0x60
#define INT_PRIORITY_RCIN		0x20

#endif
