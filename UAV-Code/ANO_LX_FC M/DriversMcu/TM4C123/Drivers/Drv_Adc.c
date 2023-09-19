#include "drv_adc.h"
#include "adc.h"
#include "hw_ints.h"

uint32_t AdcTemp;
static void ADC0Handler(void)
{
	ROM_ADCIntClear(ADC0_BASE, 0);
    ROM_ADCSequenceDataGet(ADC0_BASE, 0, &AdcTemp);
}
/**********************************************************************************************************
*函 数 名: ADC_Init
*功能说明: ADC模块初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void DrvAdcInit(void)   
{    
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	/*等待ADC模块初始化完成*/
	while(!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)); 
	ROM_GPIOPinTypeADC(ADC_PORT, ADC_PIN);
	/*使能第一个序列捕获通道的值 */
	ROM_ADCSequenceConfigure(ADC0_BASE,0,ADC_TRIGGER_PROCESSOR, 0);
	/*使能模数转换器输入0 */	
	ROM_ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH0 |ADC_CTL_END | ADC_CTL_IE );     
	ADCIntRegister(ADC0_BASE, 0, ADC0Handler);
	ROM_IntPrioritySet( INT_ADC0SS0 , USER_INT6);
	ROM_ADCIntEnable(ADC0_BASE,0); 
	ROM_ADCSequenceEnable(ADC0_BASE, 0);   
	ROM_ADCIntClear(ADC0_BASE,0);
}   

void drvAdcTrigger(void)
{
	//启动adc
	ROM_ADCProcessorTrigger(ADC0_BASE, 0); 
}

float Drv_AdcGetBatVot(void)
{
	drvAdcTrigger();
	return (AdcTemp*3.3/0xFFF)*11;
}
