#ifndef _DRV_SPI_H_
#define _DRV_SPI_H_
#include "sysconfig.h"

void DrvSSSpiInit(void);
uint8_t DrvSSSpiRTOneByte(uint8_t SendData);
void DrvSSSpiTxMultByte(uint8_t *ucp_Data, uint16_t us_Size);
void DrvSSSpiRxMultByte(uint8_t *ucp_Data, uint16_t us_Size);

#endif
