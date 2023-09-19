#include "Drv_Timer.h"
#include "timer.h"
#include "hw_ints.h"
void Timer0Irq(void)
{
	TimerIntClear(TIMER0_BASE,TIMER_TIMA_TIMEOUT); //将A通道中断标志位清除
	ANO_LX_Task();
}
void DrvTimerFcInit(void)
{
	//使能TIME0外设
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	//将Timer0A配置为32位定期计时器。
    ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    //配置定时器装载值
    ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() / 1000);
	//配置 TIMER0A 中断事件为定时器超时
	ROM_IntPrioritySet( INT_TIMER0A , USER_INT7);
    ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	TimerIntRegister(TIMER0_BASE,TIMER_A,Timer0Irq);
	//使能Timer0A.
    ROM_TimerEnable(TIMER0_BASE, TIMER_A);
}
