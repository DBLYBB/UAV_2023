#include "Drv_PwmOut.h"
#include "pwm.h"
#include "hw_types.h"
#include "hw_gpio.h"

void DrvPwmOutInit(void)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);	
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);	
	ROM_SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOA );
	ROM_SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOB );
	ROM_SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOF );
	/* Set divider to 80M/64=0.8us 精度为0.8*/
	ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_64); 
	ROM_SysCtlDelay(2);
	/*GPIO口配置*/
	ROM_GPIOPinConfigure(M0TO_PWM1_FUNCTION);
	ROM_GPIOPinConfigure(M0TO_PWM2_FUNCTION);
	ROM_GPIOPinConfigure(M0TO_PWM3_FUNCTION);
	ROM_GPIOPinConfigure(M0TO_PWM4_FUNCTION);
	ROM_GPIOPinTypePWM(GPIOB_BASE, GPIO_PIN_6);//M0PWM0
	ROM_GPIOPinTypePWM(GPIOB_BASE, GPIO_PIN_7);//M0PWM1
	ROM_GPIOPinTypePWM(GPIOB_BASE, GPIO_PIN_4);//M0PWM2
	ROM_GPIOPinTypePWM(GPIOB_BASE, GPIO_PIN_5);//M0PWM3
	/*PF0解锁操作*/
	HWREG(GPIOF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; 
	HWREG(GPIOF_BASE + GPIO_O_CR) = GPIO_PIN_0;
	HWREG(GPIOF_BASE + GPIO_O_LOCK) = 0x00;
	ROM_GPIOPinConfigure(M0TO_PWM5_FUNCTION);
	ROM_GPIOPinConfigure(M0TO_PWM6_FUNCTION);
	ROM_GPIOPinConfigure(M0TO_PWM7_FUNCTION);
	ROM_GPIOPinConfigure(M0TO_PWM8_FUNCTION);
	ROM_GPIOPinTypePWM(GPIOF_BASE, GPIO_PIN_0);//M1PWM4
	ROM_GPIOPinTypePWM(GPIOF_BASE, GPIO_PIN_1);//M1PWM5
	ROM_GPIOPinTypePWM(GPIOF_BASE, GPIO_PIN_2);//M1PWM6
	ROM_GPIOPinTypePWM(GPIOF_BASE, GPIO_PIN_3);//M1PWM7
	
	ROM_GPIOPinConfigure(HEAT_PWM_FUNCTION);
	ROM_GPIOPinTypePWM(GPIOA_BASE, GPIO_PIN_7);//M1PWM3
	/*将PWM发生器配置为倒计时模式，并立即更新参数*/
	ROM_PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
	ROM_PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
	ROM_PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
	ROM_PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
	ROM_PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
	/*周期为0.8us*3125=2500us=2.5ms(400 Hz)*/
	ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, PWM_PERIOD_MAX); 
	ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, PWM_PERIOD_MAX);
	ROM_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, PWM_PERIOD_MAX); 
	ROM_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, PWM_PERIOD_MAX); 
	ROM_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, PWM_PERIOD_MAX);
	/*使能定时器*/	
	ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_0);
	ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_1);
	ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_1);
	ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_2);
	ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_3);
	/* 使能输出 */
	ROM_PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT | PWM_OUT_2_BIT | PWM_OUT_3_BIT, true);
	ROM_PWMOutputState(PWM1_BASE, PWM_OUT_4_BIT | PWM_OUT_5_BIT | PWM_OUT_6_BIT | PWM_OUT_7_BIT, true);
	
	ROM_PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, true);
	
	/* 拉低所有输出 */	
	s16 tmp[8] = {0};
	DrvMotorPWMSet(tmp);
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
	/*使用PWM协议解析*/
	u16 tempval[8];
	for(u8 i=0; i<8; i++)
	{
		if(pwm[i]>999) pwm[i] = 999;
		tempval[i] = 1.25f*pwm[i]+1250.0f;//0-1000对应1250-2500
	}
	/*配置比较捕获寄存器的预装载值*/
	ROM_PWMPulseWidthSet(PWM0_BASE,PWM_OUT_0,tempval[0]);
	ROM_PWMPulseWidthSet(PWM0_BASE,PWM_OUT_1,tempval[1]);
	ROM_PWMPulseWidthSet(PWM0_BASE,PWM_OUT_2,tempval[2]);
	ROM_PWMPulseWidthSet(PWM0_BASE,PWM_OUT_3,tempval[3]);
	ROM_PWMPulseWidthSet(PWM1_BASE,PWM_OUT_4,tempval[4]);
	ROM_PWMPulseWidthSet(PWM1_BASE,PWM_OUT_5,tempval[5]);
	ROM_PWMPulseWidthSet(PWM1_BASE,PWM_OUT_6,tempval[6]);
	ROM_PWMPulseWidthSet(PWM1_BASE,PWM_OUT_7,tempval[7]);
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
	u16 tmpval = PWM_PERIOD_MAX  * val / 1000;
	if(tmpval > (PWM_PERIOD_MAX-1))
		tmpval = (PWM_PERIOD_MAX-1);
	ROM_PWMPulseWidthSet(PWM1_BASE,PWM_OUT_3,tmpval);	
}
