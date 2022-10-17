/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2018, hathach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#include "tusb_option.h"

#define CFG_TUSB_MCU OPT_MCU_CIU98

#if CFG_TUD_ENABLED && CFG_TUSB_MCU == OPT_MCU_CIU98

#include "device/dcd.h"
#include "core_sc000.h

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+
#define USBCON   (*((volatile unsigned char *)(0x50002000 + 0x00 )))
#define USBCMD   (*((volatile unsigned char *)(0x50002000 + 0x04 )))
#define USBDATA  (*((volatile unsigned char *)(0x50002000 + 0x08 )))
#define SYSCLKEN (*((volatile unsigned long *)(0x50007000 + 0x200)))

#define USBD_IRQn 26

enum
{
  USBEN = 0x8000, // Clock for USBD
};

enum
{
  BUSRST  = 0x01,
  SUSPEND = 0x02,
  RESUME  = 0x04,
  USBIRQ  = 0x20,
};

enum
{
  SET_USB_MODE  = 0xC0,
  SET_ADDRESS   = 0xC1,
  SET_EP_ENABLE = 0xC2,
  READ_INT_REG  = 0xC3,
  SEND_RESUME   = 0xC4,
  READ_BUF      = 0x80,
  WRITE_BUF     = 0x80,
  VALIDATE_BUF  = 0x81,
  CLEAR_BUF     = 0x82,
  ACK_SETUP     = 0x83,
};

enum
{
  PAD_STATUS = 1,
  SOFT_CON   = 2,
  SPEED_SEL  = 4,
  INT_MODE   = 8,
};

enum
{
  EP0OUT = 0x00,
  EP0IN  = 0x01,
  EP1OUT = 0x02,
  EP1IN  = 0x03,
  EP2OUT = 0x04,
  EP2IN  = 0x05,
  EP3OUT = 0x06,
  EP3IN  = 0x07,
};

enum
{
  EP_OK = 0x01,
  SETUP = 0x20,
};

#define EP_STATUS(idx)         (0x40 + (idx))

typedef struct {
  uint8_t addr;        // Endpoint address
  uint8_t index;       // Endpoint index
  uint8_t num;         // Endpoint number
  uint8_t is_in;       // Endpoint direction
  uint8_t is_stall;    // Endpoint stall condition
  uint32_t maxpacket;  // Endpoint Max packet size
  uint8_t *xfer_buff;  // Pointer to transfer buffer
  uint32_t xfer_len;   // Remained transfer length
  uint32_t xfer_count; // Current transfer length
} xfer_td_t;
static xfer_td_t xfer[4][2];

// helper getting td
static inline xfer_td_t* get_td(uint8_t epnum, uint8_t dir)
{
  return &xfer[epnum][dir];
}

/*------------------------------------------------------------------*/
/* Device API
 *------------------------------------------------------------------*/

// Initialize controller to device mode
void dcd_init (uint8_t rhport)
{
  (void) rhport;

  SYSCLKEN |= USBEN;     // Enable USB clock
  USBCMD = SET_USB_MODE; // Enable USB
  USBDATA = INT_MODE | SPEED_SEL | SOFT_CON | PAD_STATUS;
}

// Enable device interrupt
void dcd_int_enable (uint8_t rhport)
{
  (void) rhport;

  NVIC_SetPriority(USBD_IRQn, 1, 0);
  NVIC_EnableIRQ(USBD_IRQn);
}

// Disable device interrupt
void dcd_int_disable (uint8_t rhport)
{
  (void) rhport;

  NVIC_DisableIRQ(USBD_IRQn);
}

// Receive Set Address request, mcu port must also include status IN response
void dcd_set_address (uint8_t rhport, uint8_t dev_addr)
{
  (void) rhport;
  (void) dev_addr;

  USBCMD = SET_ADDRESS;
  USBDATA = dev_addr | 0x80;

  // Response with zlp status
  dcd_edpt_xfer(rhport, 0x80, NULL, 0);
}

// Wake up host
void dcd_remote_wakeup (uint8_t rhport)
{
  (void) rhport;
}

// Connect by enabling internal pull-up resistor on D+/D-
void dcd_connect(uint8_t rhport)
{
  (void) rhport;

  USBCMD = SET_USB_MODE;
  USBDATA |= SOFT_CON;
}

// Disconnect by disabling internal pull-up resistor on D+/D-
void dcd_disconnect(uint8_t rhport)
{
  (void) rhport;

  USBCMD = SET_USB_MODE;
  USBDATA &= ~SOFT_CON;
}

void dcd_sof_enable(uint8_t rhport, bool en)
{
  (void) rhport;
  (void) en;

  // TODO implement later
}

//--------------------------------------------------------------------+
// Endpoint API
//--------------------------------------------------------------------+

// Configure endpoint's registers according to descriptor
bool dcd_edpt_open (uint8_t rhport, tusb_desc_endpoint_t const * ep_desc)
{
  (void) rhport;
  (void) ep_desc;
  return false;
}

void dcd_edpt_close_all (uint8_t rhport)
{
  (void) rhport;
}

// Submit a transfer, When complete dcd_event_xfer_complete() is invoked to notify the stack
bool dcd_edpt_xfer (uint8_t rhport, uint8_t ep_addr, uint8_t * buffer, uint16_t total_bytes)
{
  (void) rhport;
  (void) ep_addr;

  uint8_t epnum = tu_edpt_number(ep_addr);
  uint8_t dir   = tu_edpt_dir(ep_addr);
  xfer_td_t* td = get_td(epnum, dir);

  td->xfer_buff = buffer;
  td->xfer_len = total_bytes;

  uint16_t len = total_bytes;
  if (len > td->maxpacket)
  {
    td->xfer_len -= len;
  }

  USBCMD = td->index;
  USBCMD = CLEAR_BUF;
  USBCMD = WRITE_BUF;
  while (len-- > 0)
  {
    USBDATA = *td->xfer_buff++;
  }
  USBCMD = VALIDATE_BUF;

  return true;
}

// Submit a transfer where is managed by FIFO, When complete dcd_event_xfer_complete() is invoked to notify the stack - optional, however, must be listed in usbd.c
bool dcd_edpt_xfer_fifo (uint8_t rhport, uint8_t ep_addr, tu_fifo_t * ff, uint16_t total_bytes)
{
  (void) rhport;
  (void) ep_addr;
  (void) ff;
  (void) total_bytes;
  return false;
}

// Stall endpoint
void dcd_edpt_stall (uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;
  (void) ep_addr;
}

// clear stall, data toggle is also reset to DATA0
void dcd_edpt_clear_stall (uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;
  (void) ep_addr;
}

/*------------------------------------------------------------------*/
/* Interrupt Handler
 *------------------------------------------------------------------*/

static void bus_reset(void)
{
}

static void ep_in_handler(uint8_t ep_addr)
{
  uint8_t epnum = tu_edpt_number(ep_addr);
  uint8_t dir   = tu_edpt_dir(ep_addr);
  xfer_td_t* td = get_td(epnum, dir);

  USBCMD = EP_STATUS(td->index);
  uint8_t status = USBDATA;
  if (!(status & EP_OK))
  {
    // TODO: print error
    return;
  }

  if (td->xfer_len == 0)
  {
    // TODO: call tinyusb
  }else
  {
    dcd_edpt_xfer(0, ep_addr, td->xfer_buff, td->xfer_len);
  }
}

static void ep_out_handler(uint8_t ep_addr)
{
  uint8_t epnum = tu_edpt_number(ep_addr);
  uint8_t dir   = tu_edpt_dir(ep_addr);
  xfer_td_t* td = get_td(epnum, dir);

  USBCMD = EP_STATUS(td->index);
  uint8_t status = USBDATA;
  if (!(status & EP_OK))
  {
    // TODO: print error
    return;
  }

  if (status & SETUP)
  {
    // TODO read EP to where?
  }else
  {
    // TODO read
  }
}

void dcd_int_handler(uint8_t rhport)
{
  (void) rhport;

  if (!(USBCON & USBIRQ)) return;

  USBCMD = READ_INT_REG;
  uint8_t irq = USBDATA;
  if (irq & BUSRST)
  {
    irq = USBDATA;
    bus_reset();
  }else
  {
    irq = USBDATA;
    if (irq & (1 << EP0OUT))
    {
      ep_out_handler(0x00);
    }else if (irq & (1 << EP0IN))
    {
      ep_in_handler(0x80);
    }else if (irq & (1 << EP1OUT))
    {
      ep_out_handler(0x01);
    }else if (irq & (1 << EP1IN))
    {
      ep_in_handler(0x81);
    }else if (irq & (1 << EP2OUT))
    {
      ep_out_handler(0x02);
    }else if (irq & (1 << EP2IN))
    {
      ep_in_handler(0x82);
    }else if (irq & (1 << EP3OUT))
    {
      ep_out_handler(0x03);
    }else if (irq & (1 << EP3IN))
    {
      ep_in_handler(0x83);
    }
  }
}

#endif
