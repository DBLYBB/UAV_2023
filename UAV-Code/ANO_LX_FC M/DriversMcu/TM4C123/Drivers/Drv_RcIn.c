#include "Drv_RcIn.h"
#include "Timer.h"
#include "hw_ints.h"
#include "uart.h"

static void PPM_Decode(void)
{
	static uint32_t	PeriodVal1,PeriodVal2 = 0;
	static uint32_t PulseHigh;
	/*����жϱ�־*/
	ROM_TimerIntClear( WTIMER1_BASE , TIMER_CAPA_EVENT );
	/*��ȡ����ֵ*/	
	PeriodVal1 = ROM_TimerValueGet( WTIMER1_BASE , TIMER_A );
	if( PeriodVal1 > PeriodVal2 )
		PulseHigh =  (PeriodVal1 - PeriodVal2) /80;
	else
		PulseHigh =  (PeriodVal1  - PeriodVal2 + 0xffffff)/80;
	PeriodVal2 = PeriodVal1;
	DrvPpmGetOneCh(PulseHigh);
}
void DrvRcPpmInit(void)
{
	ROM_SysCtlPeripheralEnable(PPM_SYSCTL);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER1);
	/*GPIOC����Ϊ��ʱ������ģʽ*/
	ROM_GPIOPinTypeTimer(PPM_PORTS, PPM_PIN);
	ROM_GPIOPinConfigure(PPM_FUNCTION);
	/*���ö�ʱ��5BΪ����������*/
	ROM_TimerConfigure( WTIMER1_BASE ,TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP ); 
	ROM_TimerControlEvent(WTIMER1_BASE,TIMER_A,TIMER_EVENT_POS_EDGE);	
	ROM_TimerLoadSet( WTIMER1_BASE , TIMER_A , 0xffff );
	ROM_TimerPrescaleSet( WTIMER1_BASE , TIMER_A , 0xff );
	/*������ʱ���ж�*/
	TimerIntRegister(WTIMER1_BASE,  TIMER_A , PPM_Decode);	
	ROM_IntPrioritySet( INT_WTIMER1A , USER_INT6);
	ROM_TimerIntEnable( WTIMER1_BASE , TIMER_CAPA_EVENT);
	ROM_TimerEnable( WTIMER1_BASE, TIMER_A );
	ROM_IntEnable( INT_WTIMER1A );
}

static void Sbus_IRQHandler(void)
{
	uint8_t com_data;	
	/*��ȡ�жϱ�־ ԭʼ�ж�״̬ �����жϱ�־*/		
	uint32_t flag = ROM_UARTIntStatus(SBUS_UART,1);
	/*����жϱ�־*/
	ROM_UARTIntClear(SBUS_UART,flag);
	ROM_UARTRxErrorClear( SBUS_UART );
	/*�ж�FIFO�Ƿ�������*/
	while(ROM_UARTCharsAvail(SBUS_UART))			
	{		
		com_data=UART3->DR;
		DrvSbusGetOneByte(com_data);	
	}
}
void DrvRcSbusInit(void)
{
	ROM_SysCtlPeripheralEnable(SBUS_SYSCTL);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);
	/*GPIO��UARTģʽ����*/
	ROM_GPIOPinConfigure(UART3_RX);
	ROM_GPIOPinTypeUART( UART3_PORT ,UART3_PIN_RX );
	/*���ô��ڵĲ����ʺ�ʱ��Դ*/
	ROM_UARTConfigSetExpClk( SBUS_UART ,SysCtlClockGet(), SBUS_BAUDRATE ,UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_TWO | UART_CONFIG_PAR_EVEN );
	/*FIFO����*/	
	ROM_UARTFIFOLevelSet( SBUS_UART , UART_FIFO_TX1_8 , UART_FIFO_RX1_8 );
	ROM_UARTFIFOEnable(SBUS_UART);
	/*ʹ�ܴ���*/
	ROM_UARTEnable( SBUS_UART );
	/*�����ж�������ʹ��*/		
	UARTIntRegister( SBUS_UART , Sbus_IRQHandler );
	ROM_IntPrioritySet( INT_UART3 , USER_INT6 );
	ROM_UARTIntEnable( SBUS_UART , UART_INT_RX | UART_INT_OE );
}
