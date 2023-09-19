#include "Drv_Uart.h"
#include "Drv_UbloxGPS.h"
#include "ANO_DT_LX.h"
#include "Drv_AnoOf.h"
/////////////////////////////////////////////////////////////////////////////////////////////////
void NoUse(u8 data){}
//串口接收发送快速定义，直接修改此处的函数名称宏，修改成自己的串口解析和发送函数名称即可，注意函数参数格式需统一
#define U1GetOneByte	UBLOX_M8_GPS_Data_Receive
#define U2GetOneByte	ANO_DT_LX_Data_Receive_Prepare
#define U3GetOneByte	NoUse
#define U4GetOneByte	AnoOF_GetOneByte
#define U5GetOneByte	NoUse	//注意MSP432只有4个串口，串口5禁止使用
/////////////////////////////////////////////////////////////////////////////////////////////////
u8 U1TxDataTemp[256];
u8 U1TxInCnt = 0;
u8 U1TxOutCnt = 0;
void DrvUart1Init(uint32_t baudrate)
{
	//注意：MSP432串口波特率设置比较特殊，建议使用如下网址进行计算，这里不再使用函数参数
	//http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
	eUSCI_UART_ConfigV1 uartConfig500000 =
	{
		EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
		6,                       				// BRDIV
		0,                                       // UCxBRF
		0,                                      // UCxBRS 
		EUSCI_A_UART_NO_PARITY,                  // No Parity
		EUSCI_A_UART_LSB_FIRST,                  // MSB First
		EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
		EUSCI_A_UART_MODE,                       // UART mode
		EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION,  // Oversampling
		EUSCI_A_UART_8_BIT_LEN                  // 8 bit data length
	};
	eUSCI_UART_ConfigV1 uartConfig115200 =
	{
		EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
		26,                       				// BRDIV
		1,                                       // UCxBRF
		0,                                      // UCxBRS 
		EUSCI_A_UART_NO_PARITY,                  // No Parity
		EUSCI_A_UART_LSB_FIRST,                  // MSB First
		EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
		EUSCI_A_UART_MODE,                       // UART mode
		EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION,  // Oversampling
		EUSCI_A_UART_8_BIT_LEN                  // 8 bit data length
	};
	eUSCI_UART_ConfigV1 uartConfig38400 =
	{
		EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
		78,                       				// BRDIV
		2,                                       // UCxBRF
		0,                                      // UCxBRS 
		EUSCI_A_UART_NO_PARITY,                  // No Parity
		EUSCI_A_UART_LSB_FIRST,                  // MSB First
		EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
		EUSCI_A_UART_MODE,                       // UART mode
		EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION,  // Oversampling
		EUSCI_A_UART_8_BIT_LEN                  // 8 bit data length
	};
	eUSCI_UART_ConfigV1 uartConfig19200 =
	{
		EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
		156,                       				// BRDIV
		4,                                       // UCxBRF
		0,                                      // UCxBRS 
		EUSCI_A_UART_NO_PARITY,                  // No Parity
		EUSCI_A_UART_LSB_FIRST,                  // MSB First
		EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
		EUSCI_A_UART_MODE,                       // UART mode
		EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION,  // Oversampling
		EUSCI_A_UART_8_BIT_LEN                  // 8 bit data length
	};
	eUSCI_UART_ConfigV1 uartConfig9600 =
	{
		EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
		312,                       				// BRDIV
		8,                                       // UCxBRF
		0,                                      // UCxBRS 
		EUSCI_A_UART_NO_PARITY,                  // No Parity
		EUSCI_A_UART_LSB_FIRST,                  // MSB First
		EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
		EUSCI_A_UART_MODE,                       // UART mode
		EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION,  // Oversampling
		EUSCI_A_UART_8_BIT_LEN                  // 8 bit data length
	};
	MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
	/* Configuring UART Module */
	if(baudrate == 9600)
		MAP_UART_initModule(EUSCI_A0_BASE, &uartConfig9600);
	if(baudrate == 19200)
		MAP_UART_initModule(EUSCI_A0_BASE, &uartConfig19200);
	if(baudrate == 38400)
		MAP_UART_initModule(EUSCI_A0_BASE, &uartConfig38400);
	if(baudrate == 115200)
		MAP_UART_initModule(EUSCI_A0_BASE, &uartConfig115200);
	if(baudrate == 500000)
		MAP_UART_initModule(EUSCI_A0_BASE, &uartConfig500000);
	/* Enable UART module */
	MAP_UART_enableModule(EUSCI_A0_BASE);
	/* Enabling interrupts */
    MAP_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
	//MAP_UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA0);
}
void DrvUart1SendBuf(u8 *data, u8 len)
{
	for(u8 i=0; i<len; i++)
	{
		U1TxDataTemp[U1TxInCnt++] = * ( data + i );
	}
	//如果没开发送中断，则打开发送中断
	if(! (UCA0IE & EUSCI_A_UART_TRANSMIT_INTERRUPT))
		MAP_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT);
}
u8 U1RxDataTmp[100];
u8 U1RxInCnt = 0;
u8 U1RxoutCnt = 0;
void drvU1GetByte(u8 data)
{
	U1RxDataTmp[U1RxInCnt++] = data;
	if(U1RxInCnt >= 100)
		U1RxInCnt = 0;
}
void drvU1DataCheck(void)
{
	while(U1RxInCnt!=U1RxoutCnt)
	{
		U1GetOneByte(U1RxDataTmp[U1RxoutCnt++]);
		if(U1RxoutCnt >= 100)
			U1RxoutCnt = 0;
	}
}
void EUSCIA0_IRQHandler(void)
{
	u8 com_data;
	uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A0_BASE);

    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        com_data = MAP_UART_receiveData(EUSCI_A0_BASE);
        drvU1GetByte(com_data);
    }
	if(status & EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG)
    {
        MAP_UART_transmitData(EUSCI_A0_BASE, U1TxDataTemp[U1TxOutCnt++]);
		if (U1TxOutCnt == U1TxInCnt)
        {
            MAP_UART_disableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT); //关闭发送中断
        }
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////
u8 U2TxDataTemp[256];
u8 U2TxInCnt = 0;
u8 U2TxOutCnt = 0;
void DrvUart2Init(uint32_t baudrate)
{
	//注意：MSP432串口波特率设置比较特殊，建议使用如下网址进行计算，这里不再使用函数参数
	//http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
	eUSCI_UART_ConfigV1 uartConfig =
	{
		EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
		6,                       // BRDIV
		0,                                       // UCxBRF
		0,                                      // UCxBRS 
		EUSCI_A_UART_NO_PARITY,                  // No Parity
		EUSCI_A_UART_LSB_FIRST,                  // LSB First
		EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
		EUSCI_A_UART_MODE,                       // UART mode
		EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION,  // Oversampling
		EUSCI_A_UART_8_BIT_LEN                  // 8 bit data length
	};
	MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
	/* Configuring UART Module */
	MAP_UART_initModule(EUSCI_A1_BASE, &uartConfig);
	/* Enable UART module */
	MAP_UART_enableModule(EUSCI_A1_BASE);
	/* Enabling interrupts */
    MAP_UART_enableInterrupt(EUSCI_A1_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
	//MAP_UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT);
	MAP_Interrupt_setPriority(INT_EUSCIA1, INT_PRIORITY_UART);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA1);
}
void DrvUart2SendBuf(u8 *data, u8 len)
{
	for(u8 i=0; i<len; i++)
	{
		U2TxDataTemp[U2TxInCnt++] = * ( data + i );
	}
	//如果没开发送中断，则打开发送中断
	if(! (UCA1IE & EUSCI_A_UART_TRANSMIT_INTERRUPT))
		MAP_UART_enableInterrupt(EUSCI_A1_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT);
}
u8 U2RxDataTmp[100];
u8 U2RxInCnt = 0;
u8 U2RxoutCnt = 0;
void drvU2GetByte(u8 data)
{
	U2RxDataTmp[U2RxInCnt++] = data;
	if(U2RxInCnt >= 100)
		U2RxInCnt = 0;
}
void drvU2DataCheck(void)
{
	while(U2RxInCnt!=U2RxoutCnt)
	{
		U2GetOneByte(U2RxDataTmp[U2RxoutCnt++]);
		if(U2RxoutCnt >= 100)
			U2RxoutCnt = 0;
	}
}
void EUSCIA1_IRQHandler(void)
{
	u8 com_data;
	uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A1_BASE);

    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        com_data = MAP_UART_receiveData(EUSCI_A1_BASE);
        drvU2GetByte(com_data);
    }
	if(status & EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG)
    {
        MAP_UART_transmitData(EUSCI_A1_BASE, U2TxDataTemp[U2TxOutCnt++]);
		if (U2TxOutCnt == U2TxInCnt)
        {
            MAP_UART_disableInterrupt(EUSCI_A1_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT); //关闭发送中断
        }
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////
u8 U3TxDataTemp[256];
u8 U3TxInCnt = 0;
u8 U3TxOutCnt = 0;
void DrvUart3Init(uint32_t baudrate)
{
	//注意：MSP432串口波特率设置比较特殊，建议使用如下网址进行计算，这里不再使用函数参数
	//http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
	eUSCI_UART_ConfigV1 uartConfig =
	{
		EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
		6,                       // BRDIV
		0,                                       // UCxBRF
		0,                                      // UCxBRS 
		EUSCI_A_UART_NO_PARITY,                  // No Parity
		EUSCI_A_UART_LSB_FIRST,                  // MSB First
		EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
		EUSCI_A_UART_MODE,                       // UART mode
		EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION,  // Oversampling
		EUSCI_A_UART_8_BIT_LEN                  // 8 bit data length
	};
	MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3,GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
	/* Configuring UART Module */
	MAP_UART_initModule(EUSCI_A2_BASE, &uartConfig);
	/* Enable UART module */
	MAP_UART_enableModule(EUSCI_A2_BASE);
	/* Enabling interrupts */
    MAP_UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
	//MAP_UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA2);
}
void DrvUart3SendBuf(u8 *data, u8 len)
{
	u8 i;
    for (i = 0; i < len; i++)
    {
        U3TxDataTemp[U3TxInCnt++] = *(data + i);
    }
	
	//如果没开发送中断，则打开发送中断
	if(! (UCA2IE & EUSCI_A_UART_TRANSMIT_INTERRUPT))
		MAP_UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT);
}
u8 U3RxDataTmp[100];
u8 U3RxInCnt = 0;
u8 U3RxoutCnt = 0;
void drvU3GetByte(u8 data)
{
	U3RxDataTmp[U3RxInCnt++] = data;
	if(U3RxInCnt >= 100)
		U3RxInCnt = 0;
}
void drvU3DataCheck(void)
{
	while(U3RxInCnt!=U3RxoutCnt)
	{
		U3GetOneByte(U3RxDataTmp[U3RxoutCnt++]);
		if(U3RxoutCnt >= 100)
			U3RxoutCnt = 0;
	}
}
void EUSCIA2_IRQHandler(void)
{
	u8 com_data;
	uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A2_BASE);

    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        com_data = MAP_UART_receiveData(EUSCI_A2_BASE);
        drvU3GetByte(com_data);
    }
	if(status & EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG)
    {
        MAP_UART_transmitData(EUSCI_A2_BASE, U3TxDataTemp[U3TxOutCnt++]);
		if (U3TxOutCnt == U3TxInCnt)
        {
            MAP_UART_disableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT); //关闭发送中断
        }
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////
u8 U4TxDataTemp[256];
u8 U4TxInCnt = 0;
u8 U4TxOutCnt = 0;
void DrvUart4Init(uint32_t baudrate)
{
	//注意：MSP432串口波特率设置比较特殊，建议使用如下网址进行计算，这里不再使用函数参数
	//http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
	eUSCI_UART_ConfigV1 uartConfig =
	{
		EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
		6,                       // BRDIV
		0,                                       // UCxBRF
		0,                                      // UCxBRS 
		EUSCI_A_UART_NO_PARITY,                  // No Parity
		EUSCI_A_UART_LSB_FIRST,                  // MSB First
		EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
		EUSCI_A_UART_MODE,                       // UART mode
		EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION,  // Oversampling
		EUSCI_A_UART_8_BIT_LEN                  // 8 bit data length
	};
	MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P9,GPIO_PIN6 | GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);
	/* Configuring UART Module */
	MAP_UART_initModule(EUSCI_A3_BASE, &uartConfig);
	/* Enable UART module */
	MAP_UART_enableModule(EUSCI_A3_BASE);
	/* Enabling interrupts */
    MAP_UART_enableInterrupt(EUSCI_A3_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
	//MAP_UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT);
	MAP_Interrupt_setPriority(INT_EUSCIA3, INT_PRIORITY_UART);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA3);
}
void DrvUart4SendBuf(u8 *data, u8 len)
{
	u8 i;
    for (i = 0; i < len; i++)
    {
        U4TxDataTemp[U4TxInCnt++] = *(data + i);
    }
	
	//如果没开发送中断，则打开发送中断
	if(! (UCA3IE & EUSCI_A_UART_TRANSMIT_INTERRUPT))
		MAP_UART_enableInterrupt(EUSCI_A3_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT);
}
u8 U4RxDataTmp[100];
u8 U4RxInCnt = 0;
u8 U4RxoutCnt = 0;
void drvU4GetByte(u8 data)
{
	U4RxDataTmp[U4RxInCnt++] = data;
	if(U4RxInCnt >= 100)
		U4RxInCnt = 0;
}
void drvU4DataCheck(void)
{
	while(U4RxInCnt!=U4RxoutCnt)
	{
		U4GetOneByte(U4RxDataTmp[U4RxoutCnt++]);
		if(U4RxoutCnt >= 100)
			U4RxoutCnt = 0;
	}
}
void EUSCIA3_IRQHandler(void)
{
	u8 com_data;
	uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A3_BASE);

    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        com_data = MAP_UART_receiveData(EUSCI_A3_BASE);
        drvU4GetByte(com_data);
    }
	if(status & EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG)
    {
        MAP_UART_transmitData(EUSCI_A3_BASE, U4TxDataTemp[U4TxOutCnt++]);
		if (U4TxOutCnt == U4TxInCnt)
        {
            MAP_UART_disableInterrupt(EUSCI_A3_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT); //关闭发送中断
        }
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////
u8 U5TxDataTemp[256];
u8 U5TxInCnt = 0;
u8 U5TxOutCnt = 0;
void DrvUart5Init(uint32_t baudrate)
{
	
}
void DrvUart5SendBuf(u8 *data, u8 len)
{
	
}
void drvU5DataCheck(void)
{

}
void DrvUartDataCheck(void)
{
	drvU1DataCheck();
	drvU2DataCheck();
	drvU3DataCheck();
	drvU4DataCheck();
	drvU5DataCheck();
}

