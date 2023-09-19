//*****************************************************************************
//
// usb_serial_structs.c - Data structures defining this CDC USB device.
//
// Copyright (c) 2012-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.4.178 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "hw_types.h"
#include "usb.h"
#include "usblib.h"
#include "usbcdc.h"
#include "usb-ids.h"
#include "usbdevice.h"
#include "usbdcdc.h"
#include "usb_serial_structs.h"

//*****************************************************************************
//
// The languages supported by this device.
//
//*****************************************************************************
const uint8_t g_pui8LangDescriptor[] =
{
    4,
    USB_DTYPE_STRING,
    USBShort(USB_LANG_EN_US)
};

//*****************************************************************************
//
// The manufacturer string.
//
//*****************************************************************************
const uint8_t g_pui8ManufacturerString[] =
{
    (17 + 1) * 2,
    USB_DTYPE_STRING,
    'T', 0, 'e', 0, 'x', 0, 'a', 0, 's', 0, ' ', 0, 'I', 0, 'n', 0, 's', 0,
    't', 0, 'r', 0, 'u', 0, 'm', 0, 'e', 0, 'n', 0, 't', 0, 's', 0,
};

//*****************************************************************************
//
// The product string.
//
//*****************************************************************************
const uint8_t g_pui8ProductString[] =
{
    2 + (17 * 2),
    USB_DTYPE_STRING,
    'A', 0, 'n', 0, 'o', 0, ' ', 0, 'P', 0, 'i', 0, 'o', 0, 'n', 0,
    'e', 0, 'e', 0, 'r', 0, 'P', 0, 'r', 0, 'o', 0, '_', 0, 'T', 0,
	'I', 0
};

//*****************************************************************************
//
// The serial number string.
//
//*****************************************************************************
const uint8_t g_pui8SerialNumberString[] =
{
    2 + (8 * 2),
    USB_DTYPE_STRING,
    '1', 0, '2', 0, '3', 0, '4', 0, '5', 0, '6', 0, '7', 0, '8', 0
};

//*****************************************************************************
//
// The control interface description string.
//
//*****************************************************************************
const uint8_t g_pui8ControlInterfaceString[] =
{
    2 + (21 * 2),
    USB_DTYPE_STRING,
    'A', 0, 'C', 0, 'M', 0, ' ', 0, 'C', 0, 'o', 0, 'n', 0, 't', 0,
    'r', 0, 'o', 0, 'l', 0, ' ', 0, 'I', 0, 'n', 0, 't', 0, 'e', 0,
    'r', 0, 'f', 0, 'a', 0, 'c', 0, 'e', 0
};

//*****************************************************************************
//
// The configuration description string.
//
//*****************************************************************************
const uint8_t g_pui8ConfigString[] =
{
    2 + (26 * 2),
    USB_DTYPE_STRING,
    'S', 0, 'e', 0, 'l', 0, 'f', 0, ' ', 0, 'P', 0, 'o', 0, 'w', 0,
    'e', 0, 'r', 0, 'e', 0, 'd', 0, ' ', 0, 'C', 0, 'o', 0, 'n', 0,
    'f', 0, 'i', 0, 'g', 0, 'u', 0, 'r', 0, 'a', 0, 't', 0, 'i', 0,
    'o', 0, 'n', 0
};

//*****************************************************************************
//
// The descriptor string table.
//
//*****************************************************************************
const uint8_t * const g_ppui8StringDescriptors[] =
{
    g_pui8LangDescriptor,
    g_pui8ManufacturerString,
    g_pui8ProductString,
    g_pui8SerialNumberString,
    g_pui8ControlInterfaceString,
    g_pui8ConfigString
};

#define NUM_STRING_DESCRIPTORS (sizeof(g_ppui8StringDescriptors) /            \
                                sizeof(uint8_t *))

//*****************************************************************************
//
// CDC device callback function prototypes.
//
//*****************************************************************************
uint32_t RxHandler(void *pvCBData, uint32_t ui32Event,
                   uint32_t ui32MsgValue, void *pvMsgData);
uint32_t TxHandler(void *pvCBData, uint32_t ui32Event,
                   uint32_t ui32MsgValue, void *pvMsgData);
uint32_t ControlHandler(void *pvCBData, uint32_t ui32Event,
                        uint32_t ui32MsgValue, void *pvMsgData);

//*****************************************************************************
//
// The CDC device initialization and customization structures. In this case,
// we are using USBBuffers between the CDC device class driver and the
// application code. The function pointers and callback data values are set
// to insert a buffer in each of the data channels, transmit and receive.
//
// With the buffer in place, the CDC channel callback is set to the relevant
// channel function and the callback data is set to point to the channel
// instance data. The buffer, in turn, has its callback set to the application
// function and the callback data set to our CDC instance structure.
//
//*****************************************************************************
tUSBDCDCDevice g_sCDCDevice =
{
    USB_VID_TI_1CBE,
    USB_PID_SERIAL,
    0,
    USB_CONF_ATTR_SELF_PWR,
    ControlHandler,
    (void *)&g_sCDCDevice,
    USBBufferEventCallback,
    (void *)&g_sRxBuffer,
    USBBufferEventCallback,
    (void *)&g_sTxBuffer,
    g_ppui8StringDescriptors,
    NUM_STRING_DESCRIPTORS
};

//*****************************************************************************
//
// Receive buffer (from the USB perspective).
//
//*****************************************************************************
uint8_t g_pui8USBRxBuffer[UART_BUFFER_SIZE];
tUSBBuffer g_sRxBuffer =
{
    false,                          // This is a receive buffer.
    RxHandler,                      // pfnCallback
    (void *)&g_sCDCDevice,          // Callback data is our device pointer.
    USBDCDCPacketRead,              // pfnTransfer
    USBDCDCRxPacketAvailable,       // pfnAvailable
    (void *)&g_sCDCDevice,          // pvHandle
    g_pui8USBRxBuffer,              // pui8Buffer
    UART_BUFFER_SIZE,               // ui32BufferSize
};

//*****************************************************************************
//
// Transmit buffer (from the USB perspective).
//
//*****************************************************************************
uint8_t g_pui8USBTxBuffer[UART_BUFFER_SIZE];
tUSBBuffer g_sTxBuffer =
{
    true,                           // This is a transmit buffer.
    TxHandler,                      // pfnCallback
    (void *)&g_sCDCDevice,          // Callback data is our device pointer.
    USBDCDCPacketWrite,             // pfnTransfer
    USBDCDCTxPacketAvailable,       // pfnAvailable
    (void *)&g_sCDCDevice,          // pvHandle
    g_pui8USBTxBuffer,              // pui8Buffer
    UART_BUFFER_SIZE,               // ui32BufferSize
};
