#include "Drv_Usb.h"
#include "usb.h"
#include "hw_ints.h"
#include "usblib.h"
#include "usbcdc.h"
#include "usb-ids.h"
#include "usbdevice.h"
#include "usbdcdc.h"

#include "usb_serial_structs.h"

static tLineCoding usb_linecoding = 
{
	500000 ,
	USB_CDC_STOP_BITS_1 ,
	USB_CDC_PARITY_EVEN ,
	8 ,
};

//*****************************************************************************
//
// Flags used to pass commands from interrupt context to the main loop.
//
//*****************************************************************************
#define COMMAND_PACKET_RECEIVED 0x00000001
#define COMMAND_STATUS_UPDATE   0x00000002

volatile uint32_t g_ui32Flags = 0;
char *g_pcStatus;
//*****************************************************************************
//
// Global flag indicating that a USB configuration has been set.
//
//*****************************************************************************
static volatile bool g_bUSBConfigured = false;

//*****************************************************************************
//
// Handles CDC driver notifications related to control and setup of the device.
//
// \param pvCBData is the client-supplied callback pointer for this channel.
// \param ui32Event identifies the event we are being notified about.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to perform control-related
// operations on behalf of the USB host.  These functions include setting
// and querying the serial communication parameters, setting handshake line
// states and sending break conditions.
//
// \return The return value is event-specific.
//
//*****************************************************************************
uint32_t
ControlHandler(void *pvCBData, uint32_t ui32Event,
               uint32_t ui32MsgValue, void *pvMsgData)
{

    switch(ui32Event)
    {
        case USB_EVENT_CONNECTED:
            g_bUSBConfigured = true;
            USBBufferFlush(&g_sTxBuffer);
            USBBufferFlush(&g_sRxBuffer);
            break;
        case USB_EVENT_DISCONNECTED:
            g_bUSBConfigured = false;
            break;

        //
        // Return the current serial communication parameters.
        //
        case USBD_CDC_EVENT_GET_LINE_CODING:
            *( (tLineCoding*)pvMsgData ) = usb_linecoding;
            break;

        //
        // Set the current serial communication parameters.
        //
        case USBD_CDC_EVENT_SET_LINE_CODING:
            usb_linecoding = *( (tLineCoding*)pvMsgData );
            break;

        //
        // Set the current serial communication parameters.
        //
        case USBD_CDC_EVENT_SET_CONTROL_LINE_STATE:
            //SetControlLineState((uint16_t)ui32MsgValue);
            break;

        //
        // Send a break condition on the serial line.
        //
        case USBD_CDC_EVENT_SEND_BREAK:
            //SendBreak(true);
            break;

        //
        // Clear the break condition on the serial line.
        //
        case USBD_CDC_EVENT_CLEAR_BREAK:
            //SendBreak(false);
            break;

        //
        // Ignore SUSPEND and RESUME for now.
        //
        case USB_EVENT_SUSPEND:
        case USB_EVENT_RESUME:
            break;

        //
        // We don't expect to receive any other events.  Ignore any that show
        // up in a release build or hang in a debug build.
        //
        default:
#ifdef DEBUG
            while(1);
#else
            break;
#endif

    }

    return(0);
}


//*****************************************************************************
//
// Handles CDC driver notifications related to the transmit channel (data to
// the USB host).
//
// \param ui32CBData is the client-supplied callback pointer for this channel.
// \param ui32Event identifies the event we are being notified about.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to notify us of any events
// related to operation of the transmit data channel (the IN channel carrying
// data to the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************
uint32_t
TxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue,
          void *pvMsgData)
{
    //
    // Which event have we been sent?
    //
    switch(ui32Event)
    {
        case USB_EVENT_TX_COMPLETE:
            //
            // Since we are using the USBBuffer, we don't need to do anything
            // here.
            //
            break;

        //
        // We don't expect to receive any other events.  Ignore any that show
        // up in a release build or hang in a debug build.
        //
        default:
#ifdef DEBUG
            while(1);
#else
            break;
#endif

    }
    return(0);
}

//*****************************************************************************
//
// Handles CDC driver notifications related to the receive channel (data from
// the USB host).
//
// \param ui32CBData is the client-supplied callback data value for this channel.
// \param ui32Event identifies the event we are being notified about.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to notify us of any events
// related to operation of the receive data channel (the OUT channel carrying
// data from the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************
uint32_t
RxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue,
          void *pvMsgData)
{
    uint32_t ui32Count;

    //
    // Which event are we being sent?
    //
    switch(ui32Event)
    {
        //
        // A new packet has been received.
        //
        case USB_EVENT_RX_AVAILABLE:
        {
            //
            // Feed some characters into the UART TX FIFO and enable the
            // interrupt so we are told when there is more space.
            //
            //USBUARTPrimeTransmit(USB_UART_BASE);
            //ROM_UARTIntEnable(USB_UART_BASE, UART_INT_TX);
            break;
        }

        //
        // We are being asked how much unprocessed data we have still to
        // process. We return 0 if the UART is currently idle or 1 if it is
        // in the process of transmitting something. The actual number of
        // bytes in the UART FIFO is not important here, merely whether or
        // not everything previously sent to us has been transmitted.
        //
        case USB_EVENT_DATA_REMAINING:
        {
            //
            // Get the number of bytes in the buffer and add 1 if some data
            // still has to clear the transmitter.
            //
            //ui32Count = ROM_UARTBusy(USB_UART_BASE) ? 1 : 0;
            return(ui32Count);
        }

        //
        // We are being asked to provide a buffer into which the next packet
        // can be read. We do not support this mode of receiving data so let
        // the driver know by returning 0. The CDC driver should not be sending
        // this message but this is included just for illustration and
        // completeness.
        //
        case USB_EVENT_REQUEST_BUFFER:
        {
            return(0);
        }

        //
        // We don't expect to receive any other events.  Ignore any that show
        // up in a release build or hang in a debug build.
        //
        default:
#ifdef DEBUG
            while(1);
#else
            break;
#endif
    }

    return(0);
}
//虚拟串口初始化函数
void DrvUsbInit(void)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	ROM_GPIOPinTypeUSBAnalog( GPIOD_BASE , GPIO_PIN_4 );	//PD4 USB DM
	ROM_GPIOPinTypeUSBAnalog( GPIOD_BASE , GPIO_PIN_5 );	//PD5 USB DP
	//初始化发送接收缓冲区
	USBBufferInit(&g_sTxBuffer);
	USBBufferInit(&g_sRxBuffer);
	USBStackModeSet(0, eUSBModeForceDevice, 0);	
	void* usbd_instance = USBDCDCInit(0, &g_sCDCDevice);	
	USBIntRegister( INT_USB0 , USB0DeviceIntHandler );
	ROM_IntPrioritySet( INT_USB0 , USER_INT7 );
}
//虚拟串口发送函数，需要发送数据时直接调用即可
void DrvUsbSend( const uint8_t* data , uint16_t length )
{
	if(g_bUSBConfigured)
		USBBufferWrite( &g_sTxBuffer , data , length );
}
//虚拟串口读取函数，此函数应定时调用，查看虚拟串口是否有数据收到
//函数返回读取到了多少字节的数据，注意缓冲区不要越界
uint16_t DrvUsbRead( uint8_t* data , uint16_t length )
{
	if(g_bUSBConfigured)
		return USBBufferRead( &g_sRxBuffer , data , length );
	else
		return 0;
}

