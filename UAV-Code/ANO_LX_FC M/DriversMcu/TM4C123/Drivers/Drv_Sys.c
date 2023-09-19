#include "Drv_Sys.h"

static uint64_t SysRunTimeMs = 0;

void SysTick_Init(void )
{
	ROM_SysTickPeriodSet(ROM_SysCtlClockGet()/1000);
	ROM_SysTickIntEnable();
	ROM_SysTickEnable();
}
void SysTick_Handler(void)
{
	SysRunTimeMs++;
}
uint32_t GetSysRunTimeMs(void)
{
	return SysRunTimeMs;
}
uint32_t GetSysRunTimeUs(void)
{
	return SysRunTimeMs*1000 + (SysTick->LOAD - SysTick->VAL) * 1000 / SysTick->LOAD;
}

void MyDelayMs(u32 time)
{
	ROM_SysCtlDelay(80000 * time /3);
}

void DrvSysInit(void)
{
	/*设置系统主频为80M*/
	ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |SYSCTL_OSC_MAIN);
	/*中断优先级组别设置*/
	NVIC_SetPriorityGrouping(0x03);
	/*开启浮点运算单元*/	
	ROM_FPULazyStackingEnable();
	ROM_FPUEnable();
	//滴答时钟
	SysTick_Init();
}
