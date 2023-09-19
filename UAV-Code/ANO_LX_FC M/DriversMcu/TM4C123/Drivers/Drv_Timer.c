#include "Drv_Timer.h"
#include "timer.h"
#include "hw_ints.h"
void Timer0Irq(void)
{
	TimerIntClear(TIMER0_BASE,TIMER_TIMA_TIMEOUT); //��Aͨ���жϱ�־λ���
	ANO_LX_Task();
}
void DrvTimerFcInit(void)
{
	//ʹ��TIME0����
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	//��Timer0A����Ϊ32λ���ڼ�ʱ����
    ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    //���ö�ʱ��װ��ֵ
    ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() / 1000);
	//���� TIMER0A �ж��¼�Ϊ��ʱ����ʱ
	ROM_IntPrioritySet( INT_TIMER0A , USER_INT7);
    ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	TimerIntRegister(TIMER0_BASE,TIMER_A,Timer0Irq);
	//ʹ��Timer0A.
    ROM_TimerEnable(TIMER0_BASE, TIMER_A);
}
