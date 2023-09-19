#ifndef _DRV_SYS_H_
#define _DRV_SYS_H_
#include "sysconfig.h"

#define TICK_PER_SECOND	1000
#define TICK_US	(1000000/TICK_PER_SECOND)

void DrvSysInit(void);

uint32_t GetSysRunTimeMs(void);
uint32_t GetSysRunTimeUs(void);
void MyDelayMs(u32 time);

#endif
