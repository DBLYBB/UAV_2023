#ifndef _USART_H
#define _USART_H
#include "SysConfig.h"
void DrvUart1Init(u32 br_num);
void Usart1_IRQ(void);
void DrvUart1SendBuf(unsigned char *DataToSend, u8 data_num);

extern u8 Rx_Buf[];
void DrvUart2Init(u32 br_num);
void Usart2_IRQ(void);
void DrvUart2SendBuf(unsigned char *DataToSend, u8 data_num);

void DrvUart3Init(u32 br_num);
void Usart3_IRQ(void);
void DrvUart3SendBuf(unsigned char *DataToSend, u8 data_num);

void DrvUart4Init(u32 br_num);
void Uart4_IRQ(void);
void DrvUart4SendBuf(unsigned char *DataToSend, u8 data_num);

void DrvUart5Init(u32 br_num);
void Uart5_IRQ(void);
void DrvUart5SendBuf(unsigned char *DataToSend, u8 data_num);

void DrvUartDataCheck(void);
#endif
