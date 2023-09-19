#include "Drv_Timer.h"

/* TimerA UpMode Configuration Parameter */
const Timer_A_UpModeConfig upConfig =
{
        TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_48,          // SMCLK/48 = 1MHz
        1000,                                  	// 1ms debounce period
        TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ,    // Enable CCR0 interrupt
        TIMER_A_DO_CLEAR                        // Clear value
};

void DrvTimerFcInit(void)
{
	MAP_Timer_A_configureUpMode(TIMER_A0_BASE, &upConfig);
	MAP_Interrupt_setPriority(INT_TA0_0, INT_PRIORITY_LXTIMER);
	MAP_Interrupt_enableInterrupt(INT_TA0_0);
	MAP_Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);
}

void TA0_0_IRQHandler(void)
{
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0);
	ANO_LX_Task();
}
