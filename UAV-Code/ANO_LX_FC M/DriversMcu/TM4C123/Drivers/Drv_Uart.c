#include "Drv_Uart.h"
#include "uart.h"
#include "hw_ints.h"
#include "hw_gpio.h"
#include "hw_types.h"
#include "Ano_DT_LX.h"
#include "Drv_UbloxGPS.h"
#include "Drv_AnoOf.h"
//TM4C�Ĵ���0��Ӧ�װ崮��1
//TM4C�Ĵ���2��Ӧ�װ崮��5
//TM4C�Ĵ���4��Ӧ�װ崮��2
//TM4C�Ĵ���5��Ӧ�װ崮��3
//TM4C�Ĵ���7��Ӧ�װ崮��4
//���ﶼ�Եװ�Ĵ��ڱ��Ϊ׼������Drv_Uart1Init��Ϊ�װ崮��1�ĳ�ʼ������TM4C�Ĵ���0
/////////////////////////////////////////////////////////////////////////////////////////////////
//���ڽ��շ��Ϳ��ٶ��壬ֱ���޸Ĵ˴��ĺ������ƺ꣬�޸ĳ��Լ��Ĵ��ڽ����ͷ��ͺ������Ƽ��ɣ�ע�⺯��������ʽ��ͳһ
void NoUse(u8 data){}
#define U1GetOneByte	UBLOX_M8_GPS_Data_Receive
#define U2GetOneByte	NoUse
#define U3GetOneByte	NoUse
#define U4GetOneByte	AnoOF_GetOneByte
#define U5GetOneByte	ANO_DT_LX_Data_Receive_Prepare
/////////////////////////////////////////////////////////////////////////////////////////////////
u8 U1TxDataTemp[256];
u8 U1TxInCnt = 0;
u8 U1TxOutCnt = 0;
void UART1_IRQHandler(void);
void DrvUart1TxCheck(void);
void DrvUart1Init(uint32_t baudrate)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	
	/*GPIO��UARTģʽ����*/
	ROM_GPIOPinConfigure(UART0_RX);
	ROM_GPIOPinConfigure(UART0_TX);
	ROM_GPIOPinTypeUART(UART0_PORT, UART0_PIN_TX | UART0_PIN_RX);
	/*���ô��ںŲ����ʺ�ʱ��Դ*/		
	ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), baudrate,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
	/*FIFO����*/
	ROM_UARTFIFOLevelSet(UART0_BASE,UART_FIFO_TX7_8,UART_FIFO_RX7_8);
	ROM_UARTFIFOEnable(UART0_BASE);
	/*ʹ�ܴ���*/
	ROM_UARTEnable( UART0_BASE );
	/*ʹ��UART0�����ж�*/			
	UARTIntRegister(UART0_BASE,UART1_IRQHandler);			
	ROM_IntPrioritySet(INT_UART0, USER_INT2);
	ROM_UARTTxIntModeSet(UART0_BASE,UART_TXINT_MODE_EOT);
	ROM_UARTIntEnable(UART0_BASE,UART_INT_RX | UART_INT_RT | UART_INT_TX);
}
void DrvUart1SendBuf(u8 *data, u8 len)
{
	for(u8 i=0; i<len; i++)
	{
		U1TxDataTemp[U1TxInCnt++] = * ( data + i );
	}
	DrvUart1TxCheck();
}
void DrvUart1TxCheck(void)
{
	while( (U1TxOutCnt != U1TxInCnt) && (ROM_UARTCharPutNonBlocking(UART0_BASE,U1TxDataTemp[U1TxOutCnt])) )
		U1TxOutCnt++;
}
void UART1_IRQHandler(void)
{
	uint8_t com_data;
	/*��ȡ�жϱ�־ ԭʼ�ж�״̬ �������жϱ�־*/		
	uint32_t flag = ROM_UARTIntStatus(UART0_BASE,1);
	/*����жϱ�־*/	
	ROM_UARTIntClear(UART0_BASE,flag);		
	/*�ж�FIFO�Ƿ�������*/		
	while(ROM_UARTCharsAvail(UART0_BASE))		
	{			
		com_data=ROM_UARTCharGet(UART0_BASE);
		U1GetOneByte(com_data);
	}
	if(flag & UART_INT_TX)
	{
		DrvUart1TxCheck();
	}
}
/////////////////////////////////////////////////////////////////////////////////////////////////
u8 U2TxDataTemp[256];
u8 U2TxInCnt = 0;
u8 U2TxOutCnt = 0;
void UART2_IRQHandler(void);
void DrvUart2TxCheck(void);
void DrvUart2Init(uint32_t baudrate)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART4);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	
	/*GPIO��UARTģʽ����*/
	ROM_GPIOPinConfigure(UART4_RX);
	ROM_GPIOPinConfigure(UART4_TX);
	ROM_GPIOPinTypeUART(UART4_PORT, UART4_PIN_TX | UART4_PIN_RX);
	/*���ô��ںŲ����ʺ�ʱ��Դ*/		
	ROM_UARTConfigSetExpClk(UART4_BASE, ROM_SysCtlClockGet(), baudrate,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
	/*FIFO����*/
	ROM_UARTFIFOLevelSet(UART4_BASE,UART_FIFO_TX7_8,UART_FIFO_RX7_8);
	ROM_UARTFIFOEnable(UART4_BASE);
	/*ʹ�ܴ���*/
	ROM_UARTEnable( UART4_BASE );
	/*ʹ��UART0�����ж�*/			
	UARTIntRegister(UART4_BASE,UART2_IRQHandler);			
	ROM_IntPrioritySet(INT_UART4, USER_INT2);
	ROM_UARTTxIntModeSet(UART4_BASE,UART_TXINT_MODE_EOT);
	ROM_UARTIntEnable(UART4_BASE,UART_INT_RX | UART_INT_RT | UART_INT_TX);
}
void DrvUart2SendBuf(u8 *data, u8 len)
{
	for(u8 i=0; i<len; i++)
	{
		U2TxDataTemp[U2TxInCnt++] = * ( data + i );
	}
	DrvUart2TxCheck();
}
void DrvUart2TxCheck(void)
{
	while( (U2TxOutCnt != U2TxInCnt) && (ROM_UARTCharPutNonBlocking(UART4_BASE,U2TxDataTemp[U2TxOutCnt])) )
		U2TxOutCnt++;
}
void UART2_IRQHandler(void)
{
	uint8_t com_data;
	/*��ȡ�жϱ�־ ԭʼ�ж�״̬ �������жϱ�־*/		
	uint32_t flag = ROM_UARTIntStatus(UART4_BASE,1);
	/*����жϱ�־*/	
	ROM_UARTIntClear(UART4_BASE,flag);		
	/*�ж�FIFO�Ƿ�������*/		
	while(ROM_UARTCharsAvail(UART4_BASE))		
	{			
		com_data=ROM_UARTCharGet(UART4_BASE);
		U2GetOneByte(com_data);
	}
	if(flag & UART_INT_TX)
	{
		DrvUart2TxCheck();
	}
}
/////////////////////////////////////////////////////////////////////////////////////////////////
u8 U3TxDataTemp[256];
u8 U3TxInCnt = 0;
u8 U3TxOutCnt = 0;
void UART3_IRQHandler(void);
void DrvUart3TxCheck(void);
void DrvUart3Init(uint32_t baudrate)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	
	/*GPIO��UARTģʽ����*/
	ROM_GPIOPinConfigure(UART5_RX);
	ROM_GPIOPinConfigure(UART5_TX);
	ROM_GPIOPinTypeUART(UART5_PORT, UART5_PIN_TX | UART5_PIN_RX);
	/*���ô��ںŲ����ʺ�ʱ��Դ*/		
	ROM_UARTConfigSetExpClk(UART5_BASE, ROM_SysCtlClockGet(), baudrate,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
	/*FIFO����*/
	ROM_UARTFIFOLevelSet(UART5_BASE,UART_FIFO_TX7_8,UART_FIFO_RX7_8);
	ROM_UARTFIFOEnable(UART5_BASE);
	/*ʹ�ܴ���*/
	ROM_UARTEnable( UART5_BASE );
	/*ʹ��UART0�����ж�*/			
	UARTIntRegister(UART5_BASE,UART3_IRQHandler);			
	ROM_IntPrioritySet(INT_UART5, USER_INT2);
	ROM_UARTTxIntModeSet(UART5_BASE,UART_TXINT_MODE_EOT);
	ROM_UARTIntEnable(UART5_BASE,UART_INT_RX | UART_INT_RT | UART_INT_TX);
}
void DrvUart3SendBuf(u8 *data, u8 len)
{
	for(u8 i=0; i<len; i++)
	{
		U3TxDataTemp[U3TxInCnt++] = * ( data + i );
	}
	DrvUart3TxCheck();
}
void DrvUart3TxCheck(void)
{
	while( (U3TxOutCnt != U3TxInCnt) && (ROM_UARTCharPutNonBlocking(UART5_BASE,U3TxDataTemp[U3TxOutCnt])) )
		U3TxOutCnt++;
}
void UART3_IRQHandler(void)
{
	uint8_t com_data;
	/*��ȡ�жϱ�־ ԭʼ�ж�״̬ �������жϱ�־*/		
	uint32_t flag = ROM_UARTIntStatus(UART5_BASE,1);
	/*����жϱ�־*/	
	ROM_UARTIntClear(UART5_BASE,flag);		
	/*�ж�FIFO�Ƿ�������*/		
	while(ROM_UARTCharsAvail(UART5_BASE))		
	{			
		com_data=ROM_UARTCharGet(UART5_BASE);
		U3GetOneByte(com_data);
	}
	if(flag & UART_INT_TX)
	{
		DrvUart3TxCheck();
	}
}
/////////////////////////////////////////////////////////////////////////////////////////////////
u8 U4TxDataTemp[256];
u8 U4TxInCnt = 0;
u8 U4TxOutCnt = 0;
void UART4_IRQHandler(void);
void DrvUart4TxCheck(void);
void DrvUart4Init(uint32_t baudrate)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	
	/*GPIO��UARTģʽ����*/
	ROM_GPIOPinConfigure(UART7_RX);
	ROM_GPIOPinConfigure(UART7_TX);
	ROM_GPIOPinTypeUART(UART7_PORT, UART7_PIN_TX | UART7_PIN_RX);
	/*���ô��ںŲ����ʺ�ʱ��Դ*/		
	ROM_UARTConfigSetExpClk(UART7_BASE, ROM_SysCtlClockGet(), baudrate,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
	/*FIFO����*/
	ROM_UARTFIFOLevelSet(UART7_BASE,UART_FIFO_TX7_8,UART_FIFO_RX7_8);
	ROM_UARTFIFOEnable(UART7_BASE);
	/*ʹ�ܴ���*/
	ROM_UARTEnable( UART7_BASE );
	/*ʹ��UART0�����ж�*/			
	UARTIntRegister(UART7_BASE,UART4_IRQHandler);			
	ROM_IntPrioritySet(INT_UART7, USER_INT2);
	ROM_UARTTxIntModeSet(UART7_BASE,UART_TXINT_MODE_EOT);
	ROM_UARTIntEnable(UART7_BASE,UART_INT_RX | UART_INT_RT | UART_INT_TX);
}
void DrvUart4SendBuf(u8 *data, u8 len)
{
	for(u8 i=0; i<len; i++)
	{
		U4TxDataTemp[U4TxInCnt++] = * ( data + i );
	}
	DrvUart4TxCheck();
}
void DrvUart4TxCheck(void)
{
	while( (U4TxOutCnt != U4TxInCnt) && (ROM_UARTCharPutNonBlocking(UART7_BASE,U4TxDataTemp[U4TxOutCnt])) )
		U4TxOutCnt++;
}
void UART4_IRQHandler(void)
{
	uint8_t com_data;
	/*��ȡ�жϱ�־ ԭʼ�ж�״̬ �������жϱ�־*/		
	uint32_t flag = ROM_UARTIntStatus(UART7_BASE,1);
	/*����жϱ�־*/	
	ROM_UARTIntClear(UART7_BASE,flag);		
	/*�ж�FIFO�Ƿ�������*/		
	while(ROM_UARTCharsAvail(UART7_BASE))		
	{			
		com_data=ROM_UARTCharGet(UART7_BASE);
		U4GetOneByte(com_data);
	}
	if(flag & UART_INT_TX)
	{
		DrvUart4TxCheck();
	}
}
/////////////////////////////////////////////////////////////////////////////////////////////////
u8 U5TxDataTemp[256];
u8 U5TxInCnt = 0;
u8 U5TxOutCnt = 0;
void UART5_IRQHandler(void);
void DrvUart5TxCheck(void);
void DrvUart5Init(uint32_t baudrate)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	
	/*PD7��������*/
	HWREG(UART2_PORT + GPIO_O_LOCK) = GPIO_LOCK_KEY; 
	HWREG(UART2_PORT + GPIO_O_CR) = UART2_PIN_TX;
	HWREG(UART2_PORT + GPIO_O_LOCK) = 0x00;
	/*GPIO��UARTģʽ����*/
	ROM_GPIOPinConfigure(UART2_RX);
	ROM_GPIOPinConfigure(UART2_TX);
	ROM_GPIOPinTypeUART(UART2_PORT, UART2_PIN_TX | UART2_PIN_RX);
	/*���ô��ںŲ����ʺ�ʱ��Դ*/		
	ROM_UARTConfigSetExpClk(UART2_BASE, ROM_SysCtlClockGet(), baudrate,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
	/*FIFO����*/
	ROM_UARTFIFOLevelSet(UART2_BASE,UART_FIFO_TX7_8,UART_FIFO_RX7_8);
	ROM_UARTFIFOEnable(UART2_BASE);
	/*ʹ�ܴ���*/
	ROM_UARTEnable( UART2_BASE );
	/*ʹ��UART0�����ж�*/			
	UARTIntRegister(UART2_BASE,UART5_IRQHandler);			
	ROM_IntPrioritySet(INT_UART2, USER_INT2);
	ROM_UARTTxIntModeSet(UART2_BASE,UART_TXINT_MODE_EOT);
	ROM_UARTIntEnable(UART2_BASE,UART_INT_RX | UART_INT_RT | UART_INT_TX);
}
void DrvUart5SendBuf(u8 *data, u8 len)
{
	for(u8 i=0; i<len; i++)
	{
		U5TxDataTemp[U5TxInCnt++] = * ( data + i );
	}
	DrvUart5TxCheck();
}
void DrvUart5TxCheck(void)
{
	while( (U5TxOutCnt != U5TxInCnt) && (ROM_UARTCharPutNonBlocking(UART2_BASE,U5TxDataTemp[U5TxOutCnt])) )
		U5TxOutCnt++;
}
void UART5_IRQHandler(void)
{
	uint8_t com_data;
	/*��ȡ�жϱ�־ ԭʼ�ж�״̬ �������жϱ�־*/		
	uint32_t flag = ROM_UARTIntStatus(UART2_BASE,1);
	/*����жϱ�־*/	
	ROM_UARTIntClear(UART2_BASE,flag);		
	/*�ж�FIFO�Ƿ�������*/		
	while(ROM_UARTCharsAvail(UART2_BASE))		
	{			
		com_data=ROM_UARTCharGet(UART2_BASE);
		U5GetOneByte(com_data);
	}
	if(flag & UART_INT_TX)
	{
		DrvUart5TxCheck();
	}
}

void DrvUartDataCheck(void)
{
	//TM4C�д���FIFO���Ͳ�д�˹��������ˣ����ж��Ƿ������
	DrvUart1TxCheck();
	DrvUart2TxCheck();
	DrvUart3TxCheck();
	DrvUart4TxCheck();
	DrvUart5TxCheck();
}
