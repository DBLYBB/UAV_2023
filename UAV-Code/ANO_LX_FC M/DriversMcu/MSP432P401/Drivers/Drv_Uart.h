#ifndef _DRV_UART_H_
#define _DRV_UART_H_
#include "sysconfig.h"

#define UartSendLXIMU	DrvUart2SendBuf

void DrvUart1Init(uint32_t baudrate);
void DrvUart1SendBuf(u8 *data, u8 len);
void DrvUart2Init(uint32_t baudrate);
void DrvUart2SendBuf(u8 *data, u8 len);
void DrvUart3Init(uint32_t baudrate);
void DrvUart3SendBuf(u8 *data, u8 len);
void DrvUart4Init(uint32_t baudrate);
void DrvUart4SendBuf(u8 *data, u8 len);
void DrvUart5Init(uint32_t baudrate);
void DrvUart5SendBuf(u8 *data, u8 len);

void DrvUartDataCheck(void);
#endif
