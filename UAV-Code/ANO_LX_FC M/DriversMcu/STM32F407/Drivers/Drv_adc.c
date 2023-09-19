/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
 * 作者    ：匿名科创
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 描述    ：SPI驱动
**********************************************************************************/
#include "Drv_adc.h"

uint16_t AdcValue[50] = {0};
static void initAdcDMA(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	//
	DMA_StructInit(&DMA_InitStructure);
	//
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE); /*DMA2的时钟使能*/
	while (DMA_GetCmdStatus(DMA2_Stream0) != DISABLE)
		; /*等待DMA可以配置*/

	DMA_InitStructure.DMA_Channel = DMA_Channel_0;								/*DMA通道0*/
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_BASE + 0x4C;		/*外设地址*/
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&AdcValue;				/*存取器地址*/
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;						/*方向从外设到内存*/
	DMA_InitStructure.DMA_BufferSize = 50;										/*数据传输的数量为50*/
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			/*地址不增加*/
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;						//DMA_MemoryInc_Disable;/*地址不增加*/
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; /*数据长度半字*/
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;			/*数据长度半字*/
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;							/*高优先级*/
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;								/*循环模式*/
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;						/*禁止FIFO*/
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;			/*FIFO的值*/
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;					/*单次传输*/
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;			/*单次传输*/
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);									/**/
	DMA_Cmd(DMA2_Stream0, ENABLE);												//开启DMA传输
}

void DrvAdcInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	//
	ADC_StructInit(&ADC_InitStructure);
	//
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);  //使能ADC时钟
	initAdcDMA();
	/*初始化ADC1通道3 的IO口*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;	 /*模拟输入*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;		 /*通道5*/
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; /*不带上下拉*/
	GPIO_Init(GPIOC, &GPIO_InitStructure);			 /*初始化*/
	/*通用控制寄存器的配置*/
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;		 /*DMA失能*/
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;					 /*独立模式*/
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;					 /*APB2的4分频 即84/4=21M*/
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles; /*两个采样阶段的延时5个时钟*/
	ADC_CommonInit(&ADC_CommonInitStructure);									 /*初始化*/
	/*初始化ADC1*/
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;						/*12位模式*/
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;								/*扫描模式*/
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;							/*连续转换*/
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None; /*禁止触发检测 使用软件触发*/
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;						/*右对齐*/
	ADC_InitStructure.ADC_NbrOfConversion = 1;									/*2通道 1*/
	ADC_Init(ADC1, &ADC_InitStructure);											/*初始化*/

	ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 1, ADC_SampleTime_480Cycles); /*设置规则通道3 二个序列 采样时间 */
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);							 //源数据变化时开启DMA传输
	ADC_Cmd(ADC1, ENABLE);														 /*开启转换*/
	ADC_DMACmd(ADC1, ENABLE);													 //使能ADC传输
	ADC_SoftwareStartConv(ADC1);												 /*启动软件转换*/
}

float Drv_AdcGetBatVot(void)
{
#define UP_R 10 //10K
#define DW_R 1	//1K
	//
	float tmp = 0;
	for (u8 i = 0; i < 50; i++)
	{
		tmp += AdcValue[i] * 0.02f;
	}
	//
	return tmp / 4096 * 3300 * (UP_R + DW_R) / DW_R * 0.001f; 
}
