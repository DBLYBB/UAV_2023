#include "drv_adc.h"

uint16_t AdcTemp;
void ADC14_IRQHandler(void)
{
	uint64_t status = MAP_ADC14_getEnabledInterruptStatus();
    MAP_ADC14_clearInterruptFlag(status);

    if (ADC_INT0 & status)
    {
        AdcTemp = MAP_ADC14_getResult(ADC_MEM0);
        MAP_ADC14_toggleConversionTrigger();
    }
}
/**********************************************************************************************************
*函 数 名: ADC_Init
*功能说明: ADC模块初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void DrvAdcInit(void)   
{    
	//![Single Sample Mode Configure]
    /* Initializing ADC (MCLK/1/4) */
    MAP_ADC14_enableModule();
    MAP_ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_4,
            0);
            
    /* Configuring GPIOs (5.5 A0) */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN5,
    GPIO_TERTIARY_MODULE_FUNCTION);

    /* Configuring ADC Memory */
    MAP_ADC14_configureSingleSampleMode(ADC_MEM0, true);
    MAP_ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS,
    ADC_INPUT_A0, false);

    /* Configuring Sample Timer */
    MAP_ADC14_enableSampleTimer(ADC_MANUAL_ITERATION);

    /* Enabling/Toggling Conversion */
    MAP_ADC14_enableConversion();
    MAP_ADC14_toggleConversionTrigger();
    //![Single Sample Mode Configure]

    /* Enabling interrupts */
    MAP_ADC14_enableInterrupt(ADC_INT0);
	MAP_Interrupt_setPriority(INT_ADC14, INT_PRIORITY_ADC);
    MAP_Interrupt_enableInterrupt(INT_ADC14);
}   

void drvAdcTrigger(void)
{

}

float Drv_AdcGetBatVot(void)
{
	return (AdcTemp*3.3/0x4000)*11;
}
