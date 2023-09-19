#include "Drv_PwmOut.h"


void DrvPwmOutInit(void)
{
	Timer_A_UpDownModeConfig upDownConfig =
	{
			TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock SOurce
			TIMER_A_CLOCKSOURCE_DIVIDER_48,          // SMCLK/48 = 1MHz
			2500,                           		// 2500 tick period,400hz
			TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
			TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE,    // Disable CCR0 interrupt
			TIMER_A_DO_CLEAR                        // Clear value
	};

	Timer_A_CompareModeConfig compareConfig_PWM1 =
	{
		TIMER_A_CAPTURECOMPARE_REGISTER_1,          // Use CCR1
		TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,   // Disable CCR interrupt
		TIMER_A_OUTPUTMODE_RESET_SET,              // Toggle output but
		1000                                 // Duty Cycle
	};
	Timer_A_CompareModeConfig compareConfig_PWM2 =
	{
		TIMER_A_CAPTURECOMPARE_REGISTER_2,          // Use CCR2
		TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,   // Disable CCR interrupt
		TIMER_A_OUTPUTMODE_RESET_SET,              // Toggle output but
		1000                                 // Duty Cycle
	};
	Timer_A_CompareModeConfig compareConfig_PWM3 =
	{
		TIMER_A_CAPTURECOMPARE_REGISTER_3,          // Use CCR3
		TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,   // Disable CCR interrupt
		TIMER_A_OUTPUTMODE_RESET_SET,              // Toggle output but
		1000                                 // Duty Cycle
	};
	Timer_A_CompareModeConfig compareConfig_PWM4 =
	{
		TIMER_A_CAPTURECOMPARE_REGISTER_4,          // Use CCR4
		TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,   // Disable CCR interrupt
		TIMER_A_OUTPUTMODE_RESET_SET,              // Toggle output but
		1000                                 // Duty Cycle
	};
	MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P7,
            GPIO_PIN4 + GPIO_PIN5 + GPIO_PIN6 + GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);
	/* Configuring Timer_A1 for UpDown Mode and starting */
    MAP_Timer_A_configureUpDownMode(TIMER_A1_BASE, &upDownConfig);
	MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
	
	MAP_Timer_A_initCompare(TIMER_A1_BASE, &compareConfig_PWM1);
    MAP_Timer_A_initCompare(TIMER_A1_BASE, &compareConfig_PWM2);
    MAP_Timer_A_initCompare(TIMER_A1_BASE, &compareConfig_PWM3);
	MAP_Timer_A_initCompare(TIMER_A1_BASE, &compareConfig_PWM4);
}
/**********************************************************************************************************
*函 数 名: Drv_MotorPWMSet
*功能说明: 电机PWM输出值设置
*形    参: PWM值（0-1000）
*返 回 值: 无
*备    注:方波的周期(已配置为400hz 2.5ms)
*         决定方波的占空比(按照PWM协议应该为1250/3125     ~   2500/3125)
											40%(最低油门) ~   80%(最高油门)
**********************************************************************************************************/
void DrvMotorPWMSet(int16_t pwm[8])
{
	static u16 tmp = 1000;
	/*使用PWM协议解析*/
	for(u8 i=0; i<8; i++)
	{
		if(pwm[i]>999) pwm[i] = 999;
	}
	/*配置比较捕获寄存器的预装载值*/
	TIMER_A1->CCR[1] = 1000+pwm[0];
	TIMER_A1->CCR[2] = 1000+pwm[1];
	TIMER_A1->CCR[3] = 1000+pwm[2];
	TIMER_A1->CCR[4] = 1000+pwm[3];
}
/**********************************************************************************************************
*函 数 名: Drv_HeatSet
*功能说明: 加热PWM输出值设置
*形    参: PWM值（0-1000）
*返 回 值: 无
*备    注:方波的周期(已配置为400hz 2.5ms)
**********************************************************************************************************/
void DrvHeatSet(u16 val)
{

}
