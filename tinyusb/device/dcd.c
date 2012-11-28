/*
 * dcd.c
 *
 *  Created on: Nov 27, 2012
 *      Author: hathach (thachha@live.com)
 */

/*
 * Software License Agreement (BSD License)
 * Copyright (c) 2012, hathach (thachha@live.com)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the tiny usb stack.
 */

#include "dcd.h"

// TODO refractor later
#include "descriptors.h"

#define USB_ROM_SIZE (1024*2)
uint8_t usb_RomDriver_buffer[USB_ROM_SIZE]ALIGNED(2048) /*__BSS(RAM2)*/;
USBD_HANDLE_T g_hUsb;
volatile static bool isConfigured = false;

/**************************************************************************/
/*!
    @brief Handler for the USB Configure Event
*/
/**************************************************************************/
ErrorCode_t USB_Configure_Event (USBD_HANDLE_T hUsb)
{
  USB_CORE_CTRL_T* pCtrl = (USB_CORE_CTRL_T*)hUsb;
  if (pCtrl->config_value)
  {
    #if defined(CLASS_HID)
    ASSERT_STATUS( usb_hid_configured(hUsb) );
    #endif

    #ifdef CFG_USB_CDC
    ASSERT_STATUS( usb_cdc_configured(hUsb) );
    #endif
  }

  isConfigured = true;

  return LPC_OK;
}

/**************************************************************************/
/*!
    @brief Handler for the USB Reset Event
*/
/**************************************************************************/
ErrorCode_t USB_Reset_Event (USBD_HANDLE_T hUsb)
{
  isConfigured = false;
  return LPC_OK;
}

void dcd_init()
{
  /* ROM DRIVER INIT */
  USBD_API_INIT_PARAM_T usb_param =
  {
    .usb_reg_base        = LPC_USB_BASE,
    .max_num_ep          = USB_MAX_EP_NUM,
    .mem_base            = (uint32_t) usb_RomDriver_buffer,
    .mem_size            = USB_ROM_SIZE, //USBD_API->hw->GetMemSize()

    .USB_Configure_Event = USB_Configure_Event,
    .USB_Reset_Event     = USB_Reset_Event
  };

  USB_CORE_DESCS_T DeviceDes =
  {
    .device_desc      = (uint8_t*) &USB_DeviceDescriptor,
    .string_desc      = (uint8_t*) &USB_StringDescriptor,
    .full_speed_desc  = (uint8_t*) &USB_FsConfigDescriptor,
    .high_speed_desc  = (uint8_t*) &USB_FsConfigDescriptor,
    .device_qualifier = NULL
  };

  /* Start USB hardware initialisation */
  ASSERT_STATUS(USBD_API->hw->Init(&g_hUsb, &DeviceDes, &usb_param));

    /* Initialise the class driver(s) */
  #ifdef CFG_USB_CDC
    ASSERT_STATUS( usb_cdc_init(g_hUsb, &USB_FsConfigDescriptor.CDC_CCI_Interface,
            &USB_FsConfigDescriptor.CDC_DCI_Interface, &usb_param.mem_base, &usb_param.mem_size) );
  #endif

  #ifdef CFG_CLASS_HID_KEYBOARD
    ASSERT_STATUS( usb_hid_init(g_hUsb , &USB_FsConfigDescriptor.HID_KeyboardInterface ,
            HID_KeyboardReportDescriptor, USB_FsConfigDescriptor.HID_KeyboardHID.DescriptorList[0].wDescriptorLength,
            &usb_param.mem_base , &usb_param.mem_size) );
  #endif

  #ifdef CFG_USB_HID_MOUSE
    ASSERT_STATUS( usb_hid_init(g_hUsb , &USB_FsConfigDescriptor.HID_MouseInterface    ,
            HID_MouseReportDescriptor, USB_FsConfigDescriptor.HID_MouseHID.DescriptorList[0].wDescriptorLength,
            &usb_param.mem_base , &usb_param.mem_size) );
  #endif

  /* Enable the USB interrupt */
  NVIC_EnableIRQ(USB_IRQ_IRQn);

  /* Perform USB soft connect */
  USBD_API->hw->Connect(g_hUsb, 1);
}

/**************************************************************************/
/*!
    @brief Indicates whether USB is configured or not
*/
/**************************************************************************/
bool usb_isConfigured(void)
{
  return isConfigured;
}

/**************************************************************************/
/*!
    @brief Redirect the USB IRQ handler to the ROM handler
*/
/**************************************************************************/
void USB_IRQHandler(void)
{
  USBD_API->hw->ISR(g_hUsb);
}