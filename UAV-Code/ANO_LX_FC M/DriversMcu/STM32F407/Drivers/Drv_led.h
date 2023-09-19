#ifndef _LED_H_
#define _LED_H_

#include "SysConfig.h"

//
#define LED_R_BIT 0x01
#define LED_G_BIT 0x02
#define LED_B_BIT 0x04
//
#define LED_A_BIT 0X08
#define LED_ALL_BIT 0x0f

//
#define LED4_OFF ANO_GPIO_LED->BSRRL = ANO_Pin_LED4 //BSRRL   LEVEL_H
#define LED4_ON ANO_GPIO_LED->BSRRH = ANO_Pin_LED4	//L
//
#define LED1_ON ANO_GPIO_LED->BSRRL = ANO_Pin_LED1
#define LED1_OFF ANO_GPIO_LED->BSRRH = ANO_Pin_LED1
#define LED2_ON ANO_GPIO_LED->BSRRL = ANO_Pin_LED2
#define LED2_OFF ANO_GPIO_LED->BSRRH = ANO_Pin_LED2
#define LED3_ON ANO_GPIO_LED->BSRRL = ANO_Pin_LED3
#define LED3_OFF ANO_GPIO_LED->BSRRH = ANO_Pin_LED3

/***************LED GPIO定义******************/
#define ANO_RCC_LED RCC_AHB1Periph_GPIOE
#define ANO_GPIO_LED GPIOE
#define ANO_Pin_LED4 GPIO_Pin_7
//
#define ANO_Pin_LED1 GPIO_Pin_1 //r
#define ANO_Pin_LED2 GPIO_Pin_0 //g
#define ANO_Pin_LED3 GPIO_Pin_2 //b
/*********************************************/
#define LED_NUM 4

typedef union {
	//
	s8 brightness[LED_NUM];

} _led_st;

extern _led_st led;

void DvrLedInit(void);
void LED_1ms_DRV(void);

#endif
