/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
 * 作者    ：匿名科创
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 描述    ：PWM输出
**********************************************************************************/
#include "Drv_PwmOut.h"
#include "Ano_Math.h"

#if (PWM_FRE_HZ == 466)

//18分频到 84000000/18 = 4.666M   0.2142857us
/*初始化高电平时间1000us*/
#define INIT_DUTY 4667 //
/*精度10000，每份0.2142857us*/
#define ACCURACY 10000 //总共为2142.857us
//电调控制协议为1000us-2000us高电平时间
/*设置飞控控制信号转换比例为*/
#define PWM_RADIO 4.666f //(9333 - 4667)/1000.0

#elif (PWM_FRE_HZ == 400)

//21分频到 84000000/21 = 4M   0.25us
/*初始化高电平时间1000us*/
#define INIT_DUTY 4000 //u16(1000/0.25)
/*精度10000，每份0.25us*/
#define ACCURACY 10000 //总共为2500us
//电调控制协议为1000us-2000us高电平时间
/*设置飞控控制信号转换比例为4*/
#define PWM_RADIO 4    //(8000 - 4000)/1000.0

#elif (PWM_FRE_HZ == 350)
//24分频到 84000000/24 = 3.5M   0.2739726us
/*初始化高电平时间1000us*/
#define INIT_DUTY 3500 //
/*精度10000，每份0.2739726us*/
#define ACCURACY 10000 //总共为2857.1429us
//电调控制协议为1000us-2000us高电平时间
/*设置飞控控制信号转换比例为3.65f*/
#define PWM_RADIO 3.5f //(7300 - 3650)/1000.0

#endif

void DrvPwmOutInit(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    uint16_t PrescalerValue = 0;
    u32 hz_set = ACCURACY * PWM_FRE_HZ;

    GPIO_StructInit(&GPIO_InitStructure);
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_OCStructInit(&TIM_OCInitStructure);

    hz_set = LIMIT(hz_set, 1, 84000000);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOE, ENABLE);

    /////////////////////////////////////////////////////////////////////////////
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; //GPIO_Pin_0 | GPIO_Pin_1 |
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //  GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
    //  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM5);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM5);

    /* Compute the prescaler value */
    PrescalerValue = (uint16_t)((SystemCoreClock / 2) / hz_set) - 1;
    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = ACCURACY;
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    //  /* PWM1 Mode configuration: Channel1 */
    //  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    //  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
    //  TIM_OC1Init(TIM5, &TIM_OCInitStructure);
    //  TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);

    //  /* PWM1 Mode configuration: Channel2 */
    //  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    //  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
    //  TIM_OC2Init(TIM5, &TIM_OCInitStructure);
    //  TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);

    /* PWM1 Mode configuration: Channel3 */
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
    TIM_OC3Init(TIM5, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);

    /* PWM1 Mode configuration: Channel4 */
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
    TIM_OC4Init(TIM5, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM5, ENABLE);
    TIM_Cmd(TIM5, ENABLE);
    /////////////////////////////////////////////////////////////////////////////
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);

    /* Compute the prescaler value */
    PrescalerValue = (uint16_t)((SystemCoreClock) / hz_set) - 1;
    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = ACCURACY;
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

    /* PWM1 Mode configuration: Channel1 */
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

    /* PWM1 Mode configuration: Channel2 */
    //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

    /* PWM1 Mode configuration: Channel3 */
    //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

    /* PWM1 Mode configuration: Channel4 */
    //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
    ////////////////////////////////////////////////////////////////////////////////////

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM8);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM8);

    /* Compute the prescaler value */
    PrescalerValue = (uint16_t)((SystemCoreClock) / hz_set) - 1;
    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = ACCURACY;
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

    /* PWM1 Mode configuration: Channel3 */
    //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
    TIM_OC3Init(TIM8, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

    /* PWM1 Mode configuration: Channel4 */
    //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
    TIM_OC4Init(TIM8, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_CtrlPWMOutputs(TIM8, ENABLE);
    TIM_ARRPreloadConfig(TIM8, ENABLE);
    TIM_Cmd(TIM8, ENABLE);

#if (ESC_CALI == 1)
    //校准时先给最大油门。
    TIM1->CCR4 = 2 * INIT_DUTY; //1
    TIM1->CCR3 = 2 * INIT_DUTY; //2
    TIM1->CCR2 = 2 * INIT_DUTY; //3
    TIM1->CCR1 = 2 * INIT_DUTY; //4

    TIM5->CCR4 = 2 * INIT_DUTY; //5
    TIM5->CCR3 = 2 * INIT_DUTY; //6
    TIM8->CCR4 = 2 * INIT_DUTY; //7
    TIM8->CCR3 = 2 * INIT_DUTY; //8
#endif
}

void DrvMotorPWMSet(int16_t pwm[8])
{

    //范围0-1000，转换到4000-8000，折合1000-2000us
    TIM1->CCR4 = PWM_RADIO * (pwm[0]) + INIT_DUTY; //1
    TIM1->CCR3 = PWM_RADIO * (pwm[1]) + INIT_DUTY; //2
    TIM1->CCR2 = PWM_RADIO * (pwm[2]) + INIT_DUTY; //3
    TIM1->CCR1 = PWM_RADIO * (pwm[3]) + INIT_DUTY; //4

    TIM5->CCR4 = PWM_RADIO * (pwm[4]) + INIT_DUTY; //5
    TIM5->CCR3 = PWM_RADIO * (pwm[5]) + INIT_DUTY; //6
    TIM8->CCR4 = PWM_RADIO * (pwm[6]) + INIT_DUTY; //7
    TIM8->CCR3 = PWM_RADIO * (pwm[7]) + INIT_DUTY; //8
}

/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
