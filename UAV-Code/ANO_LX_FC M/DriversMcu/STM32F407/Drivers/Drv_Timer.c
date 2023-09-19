/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
 * 作者    ：匿名科创
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 描述    ：定时器驱动
**********************************************************************************/

#include "Drv_Timer.h"
#include "Drv_led.h"

#define SYS_TIMx_IRQn TIM7_IRQn
#define SYS_TIMx TIM7
#define SYS_RCC_TIMx RCC_APB1Periph_TIM7

void TIM_CONF(u16 period_ms) //APB1  84M
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    /* 使能时钟 */
    RCC_APB1PeriphClockCmd(SYS_RCC_TIMx, ENABLE);

    TIM_DeInit(SYS_TIMx);

    /* 自动重装载寄存器周期的值(计数值) */
    TIM_TimeBaseStructure.TIM_Period = period_ms;

    /* 累计 TIM_Period个频率后产生一个更新或者中断 */
    /* 时钟预分频数为 */
    TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1;

    /* 对外部时钟进行采样的时钟分频,这里没有用到 */
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数

    TIM_TimeBaseInit(SYS_TIMx, &TIM_TimeBaseStructure);

    TIM_ClearFlag(SYS_TIMx, TIM_FLAG_Update);

    TIM_ITConfig(SYS_TIMx, TIM_IT_Update, ENABLE);

    TIM_Cmd(SYS_TIMx, ENABLE);

    RCC_APB1PeriphClockCmd(SYS_RCC_TIMx, DISABLE); /*先关闭等待使用*/
}
void TIM_NVIC()
{
    NVIC_InitTypeDef NVIC_InitStructure;

    //    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    NVIC_InitStructure.NVIC_IRQChannel = SYS_TIMx_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_TIME_P;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_TIME_S;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void DrvTimerFcInit(void)
{
    TIM_CONF(1000);
    TIM_NVIC();

    /* TIM7 重新开时钟，开始计时 */
    RCC_APB1PeriphClockCmd(SYS_RCC_TIMx, ENABLE);
}
/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
