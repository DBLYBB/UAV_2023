#include "Drv_Led.h"
void DrvLedOnOff(u16 sta);

void DvrLedInit(void)
{
	ROM_SysCtlPeripheralEnable(LED1_SYSCTL);
	ROM_SysCtlPeripheralEnable(LED2_SYSCTL);
    ROM_SysCtlPeripheralEnable(LED3_SYSCTL);
    ROM_SysCtlPeripheralEnable(LEDS_SYSCTL);
	ROM_GPIOPinTypeGPIOOutput(LED1_PORT, LED1_PIN);
	ROM_GPIOPinTypeGPIOOutput(LED2_PORT, LED2_PIN);
	ROM_GPIOPinTypeGPIOOutput(LED3_PORT, LED3_PIN);
	ROM_GPIOPinTypeGPIOOutput(LEDS_PORT, LEDS_PIN);
	//πÿ±’À˘”–LED
	DrvLedOnOff(0);
}

void DrvLedOnOff(u16 sta)
{
	if(sta & LED_R)
		ROM_GPIOPinWrite(LED2_PORT, LED2_PIN, LED2_PIN);
	else
		ROM_GPIOPinWrite(LED2_PORT, LED2_PIN, 0);
	
	if(sta & LED_G)
		ROM_GPIOPinWrite(LED1_PORT, LED1_PIN, LED1_PIN);
	else
		ROM_GPIOPinWrite(LED1_PORT, LED1_PIN, 0);
	if(sta & LED_B)
		ROM_GPIOPinWrite(LED3_PORT, LED3_PIN, LED3_PIN);
	else
		ROM_GPIOPinWrite(LED3_PORT, LED3_PIN, 0);
	if(sta & LED_S)
		ROM_GPIOPinWrite(LEDS_PORT, LEDS_PIN, 0);
	else
		ROM_GPIOPinWrite(LEDS_PORT, LEDS_PIN, LEDS_PIN);
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

