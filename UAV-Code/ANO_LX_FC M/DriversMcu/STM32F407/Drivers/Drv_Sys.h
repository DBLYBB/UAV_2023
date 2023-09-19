#ifndef _DRV_SYS_H_
#define _DRV_SYS_H_
#include "sysconfig.h"

void DrvSysInit(void);

uint32_t GetSysRunTimeMs(void);
uint32_t GetSysRunTimeUs(void);
void MyDelayMs(u32 time);

#endif

