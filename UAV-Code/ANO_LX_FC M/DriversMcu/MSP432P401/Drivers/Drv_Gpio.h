#ifndef _DRV_GPIO_H_
#define _DRV_GPIO_H_
#include "sysconfig.h"

void DrvGpioSenserCsPinInit(void);
void DrvGpioCsPinCtrlAk8975(u8 ena);
void DrvGpioCsPinCtrlSpl06(u8 ena);
void DrvGpioCsPinCtrlBmi088Acc(u8 ena);
void DrvGpioCsPinCtrlBmi088Gyr(u8 ena);

#endif
