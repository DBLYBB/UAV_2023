#ifndef _DRV_LED_H_
#define _DRV_LED_H_
#include "sysconfig.h"

typedef union {
	//
	s8 brightness[LED_NUM];

} _led_st;

extern _led_st led;

void DvrLedInit(void);
void LED_1ms_DRV(void);

#endif
