#include "Drv_Spi.h"
#include "ssi.h"

void DrvSSSpiInit(void)
{	
	ROM_SysCtlPeripheralEnable( SYSCTL_PERIPH_SSI0 );
	ROM_SysCtlPeripheralEnable(SPI0_SYSCTL);
	/*����IO��*/	
	ROM_GPIOPinTypeSSI(SPI0_PROT,SPI0_CLK_PIN|SPI0_RX_PIN|SPI0_TX_PIN);
	ROM_GPIOPinConfigure(SPI0_CLK);	
	ROM_GPIOPinConfigure(SPI0_RX);
	ROM_GPIOPinConfigure(SPI0_TX);
	/*����SPIʱ��Ϊ1Mhz*/
	ROM_SSIConfigSetExpClk(SPI0_BASE, ROM_SysCtlClockGet(), SSI_FRF_MOTO_MODE_3,  SSI_MODE_MASTER, 10000000,  8);
	/*����SSI0*/
	ROM_SSIEnable(SPI0_BASE);
}

/* SPI��д���� */
uint8_t DrvSSSpiRTOneByte(uint8_t SendData)
{
    uint32_t ui_TempData;
    uint8_t uc_ReceiveData;
    /* ��SSI FIFOд������ */
    ROM_SSIDataPut(SPI0_BASE, SendData);
    /* �ȴ�SSI��æ */
    while(ROM_SSIBusy(SPI0_BASE));
    /* ��FIFO��ȡ���� */
    ROM_SSIDataGet(SPI0_BASE, &ui_TempData);
    /* ��ȡ���ݵĵͰ�λ */
    uc_ReceiveData = ui_TempData & 0xff;
    return uc_ReceiveData;
}

void DrvSSSpiTxMultByte(uint8_t *ucp_Data, uint16_t us_Size)
{
    uint16_t i = 0;
    /* ����д������ */
    for(i = 0; i < us_Size; i++)
    {
        DrvSSSpiRTOneByte(ucp_Data[i]);
    }
}

void DrvSSSpiRxMultByte(uint8_t *ucp_Data, uint16_t us_Size)
{
    uint16_t i = 0;
    /* ������ȡ���� */
    for(i = 0; i < us_Size; i++)
    {
        ucp_Data[i] = DrvSSSpiRTOneByte(0xFF);
    }
}
