#include "Drv_RcIn.h"

//====PPM====
void DrvRcPpmInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	//==
	GPIO_StructInit(&GPIO_InitStructure);
	TIM_ICStructInit(&TIM_ICInitStructure);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PWMIN_P;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PWMIN_S;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);

	TIM3->PSC = (168 / 2) - 1;

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);

	TIM_Cmd(TIM3, ENABLE);
	TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);
}

void PPM_IRQH()
{
	
	static u16 temp_cnt[2];
	//
	if (TIM3->SR & TIM_IT_CC2)
	{
		TIM3->SR = ~TIM_IT_CC2; //TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
		TIM3->SR = ~TIM_FLAG_CC2OF;
		//==
//		if (!(GPIOC->IDR & GPIO_Pin_7))
//		{
//			temp_cnt[0] = TIM_GetCapture2(TIM3);
//		}
//		else
//		{
//			temp_cnt[1] = TIM_GetCapture2(TIM3);
//			u16 _tmp;
//			_tmp = temp_cnt[1] - temp_cnt[0];

//			DrvPpmGetOneCh(_tmp + 400);//×ª»»µ½1000-2000
//		}
		//==
		if (!(GPIOC->IDR & GPIO_Pin_7))
		{
			temp_cnt[0] = TIM_GetCapture2(TIM3);
			u16 _tmp;
			_tmp = temp_cnt[0] - temp_cnt[1];
			//
			DrvPpmGetOneCh(_tmp);//	
			//
			temp_cnt[1] = temp_cnt[0];
		}
		else
		{
			//temp_cnt[1] = TIM_GetCapture2(TIM3);

		}
	}
}
//====S-BUS====
void DrvRcSbusInit(void)
{
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_StructInit(&GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_UART6_P;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_UART6_S;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //GPIO_PuPd_UP;//
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 100000;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_2;
	USART_InitStructure.USART_Parity = USART_Parity_Even;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx;
	USART_Init(USART6, &USART_InitStructure);

	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);

	USART_Cmd(USART6, ENABLE);
}

void Sbus_IRQH(void)
{
	u8 com_data;

	if (USART_GetITStatus(USART6, USART_IT_RXNE))
	{
		USART_ClearITPendingBit(USART6, USART_IT_RXNE);
		//==
		com_data = USART6->DR;
		//
		DrvSbusGetOneByte(com_data);
	}
}

