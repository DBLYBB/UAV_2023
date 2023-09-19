#include "Drv_Led.h"
void DrvLedOnOff(u16 sta);

void DvrLedInit(void)
{
	//OB
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P9, GPIO_PIN3);
	//RGB
	MAP_GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN3);
	MAP_GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN4);
	MAP_GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN5);
}

void DrvLedOnOff(u16 sta)
{
	if(sta & LED_R)
		MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN4);
	else
		MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN4);
	
	if(sta & LED_G)
		MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN3);
	else
		MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN3);
	
	if(sta & LED_B)
		MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
	else
		MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN5);
	
	if(sta & LED_S)
		MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P9, GPIO_PIN3);
	else
		MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P9, GPIO_PIN3);
}

_led_st led;
void LED_1ms_DRV() //
{
	static u16 led_cnt[LED_NUM];
	u16 led_tmp;
	for (u8 i = 0; i < LED_NUM; i++)
	{

		if (led_cnt[i] < (s16)led.brightness[i])
		{
			//ON
			led_tmp |= (1 << i);
		}
		else
		{
			//OFF
			led_tmp &= ~(1 << i);
		}

		if (++led_cnt[i] >= 20)
		{
			led_cnt[i] = 0;
		}
	}
	//
	DrvLedOnOff(led_tmp);
}

