#include "Drv_RcIn.h"

void TA2_N_IRQHandler(void)
{
	static uint32_t	PeriodVal1,PeriodVal2 = 0;
	static uint32_t PulseHigh;
	/*捕获中断*/
	if(Timer_A_getCaptureCompareEnabledInterruptStatus(TIMER_A2_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_1))
	{
		Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_1);
		//if(Timer_A_getSynchronizedCaptureCompareInput(TIMER_A2_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_1,TIMER_A_READ_SYNCHRONIZED_CAPTURECOMPAREINPUT) == TIMER_A_CAPTURECOMPARE_INPUT_LOW)
		if(Timer_A_getSynchronizedCaptureCompareInput(TIMER_A2_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_1,TIMER_A_READ_CAPTURE_COMPARE_INPUT) == TIMER_A_CAPTURECOMPARE_INPUT_LOW)
		{
			PeriodVal1 = Timer_A_getCaptureCompareCount(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);
			if( PeriodVal1 > PeriodVal2 )
				PulseHigh =  PeriodVal1 - PeriodVal2;
			else
				PulseHigh =  PeriodVal1 + 0xffff - PeriodVal2;
			PeriodVal2 = PeriodVal1;
			DrvPpmGetOneCh(PulseHigh);
		}
	}
	/*溢出中断*/
	else if(Timer_A_getEnabledInterruptStatus(TIMER_A2_BASE))
	{
		Timer_A_clearInterruptFlag(TIMER_A2_BASE);
	}
	BITBAND_PERI(TIMER_A_CMSIS(TIMER_A2_BASE)->CCTL[1],TIMER_A_CCTLN_COV_OFS) = 0;//软件复位COV
}
void DrvRcPpmInit(void)
{
	Timer_A_ContinuousModeConfig continuousModeConfig =
	{
		TIMER_A_CLOCKSOURCE_SMCLK,           // SMCLK Clock Source
		TIMER_A_CLOCKSOURCE_DIVIDER_48,       // SMCLK/1 = 1MHz
		TIMER_A_TAIE_INTERRUPT_ENABLE,      // Disable Timer ISR
		TIMER_A_SKIP_CLEAR                   // Skup Clear Counter
	};
	Timer_A_CaptureModeConfig CapturemodeconfigP56 =
	{
		TIMER_A_CAPTURECOMPARE_REGISTER_1,                                                //uint_fast16_t captureRegister
		TIMER_A_CAPTUREMODE_FALLING_EDGE,                        //uint_fast16_t captureMode
		TIMER_A_CAPTURE_INPUTSELECT_CCIxA,                                                //uint_fast16_t captureInputSelect
		TIMER_A_CAPTURE_SYNCHRONOUS ,                                                        //uint_fast16_t synchronizeCaptureSource
		TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE,                                //uint_fast8_t captureInterruptEnable
		TIMER_A_OUTPUTMODE_TOGGLE_RESET,                                                //uint_fast16_t captureOutputMode
	};
	GPIO_setAsPeripheralModuleFunctionInputPin( GPIO_PORT_P5 , GPIO_PIN6 , GPIO_PRIMARY_MODULE_FUNCTION );
	/* Configuring Capture Mode */
	MAP_Timer_A_initCapture ( TIMER_A2_BASE , &CapturemodeconfigP56 );
	/* Configuring Continuous Mode */
    Timer_A_configureContinuousMode(TIMER_A2_BASE, &continuousModeConfig);
	/* Enabling interrupts */
	MAP_Interrupt_setPriority(INT_TA2_N, INT_PRIORITY_RCIN);
    Interrupt_enableInterrupt(INT_TA2_N);
	/* Starting the Timer_A0 in continuous mode */
    Timer_A_startCounter(TIMER_A2_BASE, TIMER_A_CONTINUOUS_MODE);
}

static void Sbus_IRQHandler(void)
{

}
void DrvRcSbusInit(void)
{

}
