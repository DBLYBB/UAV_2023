#ifndef __ANO_LED_H
#define __ANO_LED_H
//==ÒýÓÃ
#include "sysconfig.h"

void DrvUsbInit(void);
void DrvUsbSend( const uint8_t* data , uint16_t length );
uint16_t DrvUsbRead( uint8_t* data , uint16_t length );

#endif
