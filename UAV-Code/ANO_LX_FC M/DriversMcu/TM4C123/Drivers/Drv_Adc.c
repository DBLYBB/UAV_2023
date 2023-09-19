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
*�� �� ��: ADC_Init
*����˵��: ADCģ���ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void DrvAdcInit(void)   
{    
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	/*�ȴ�ADCģ���ʼ�����*/
	while(!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)); 
	ROM_GPIOPinTypeADC(ADC_PORT, ADC_PIN);
	/*ʹ�ܵ�һ�����в���ͨ����ֵ */
	ROM_ADCSequenceConfigure(ADC0_BASE,0,ADC_TRIGGER_PROCESSOR, 0);
	/*ʹ��ģ��ת��������0 */	
	ROM_ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH0 |ADC_CTL_END | ADC_CTL_IE );     
	ADCIntRegister(ADC0_BASE, 0, ADC0Handler);
	ROM_IntPrioritySet( INT_ADC0SS0 , USER_INT6);
	ROM_ADCIntEnable(ADC0_BASE,0); 
	ROM_ADCSequenceEnable(ADC0_BASE, 0);   
	ROM_ADCIntClear(ADC0_BASE,0);
}   

void drvAdcTrigger(void)
{
	//����adc
	ROM_ADCProcessorTrigger(ADC0_BASE, 0); 
}

float Drv_AdcGetBatVot(void)
{
	drvAdcTrigger();
	return (AdcTemp*3.3/0xFFF)*11;
}
