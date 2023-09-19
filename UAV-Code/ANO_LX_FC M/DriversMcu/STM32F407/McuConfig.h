#ifndef _MCUCONFIG_H_
#define _MCUCONFIG_H_
#include "stm32f4xx.h"

//=======================================
/***************中断优先级******************/
#define NVIC_GROUP NVIC_PriorityGroup_3 //中断分组选择

#define NVIC_PWMIN_P 2 //接收机采集中断配置
#define NVIC_PWMIN_S 1

#define NVIC_TIME_P 7 //定时器中断配置
#define NVIC_TIME_S 1

#define NVIC_UART6_P 4 //串口6中断配置
#define NVIC_UART6_S 1

#define NVIC_UART5_P 1 //串口5中断配置
#define NVIC_UART5_S 0

#define NVIC_UART4_P 3 //串口4中断配置
#define NVIC_UART4_S 1

#define NVIC_UART2_P 4 //串口2中断配置
#define NVIC_UART2_S 1

#define NVIC_UART1_P 3 //串口1中断配置 //gps
#define NVIC_UART1_S 0
/***********************************************/
//=========================================

#define UartSendLXIMU 	DrvUart5SendBuf
#endif

