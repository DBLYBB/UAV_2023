#ifndef __DRV_RC_INPUT_H
#define __DRV_RC_INPUT_H

#include "SysConfig.h"

void DrvRcPpmInit(void);
void DrvRcSbusInit(void);
void PPM_IRQH(void);
void Sbus_IRQH(void);

#endif
