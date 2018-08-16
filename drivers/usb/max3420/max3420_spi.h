/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation; version 2.1 of the
    License.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

    Copyright (c) 2016 Sebastien Cerdan <sebcerdan@gmail.com>

*/

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>
#include <mutek/bytecode.h>

#include <hexo/bit.h>
#include <hexo/endian.h>

#include <device/resources.h>
#include <device/device.h>
#include <device/driver.h>
#include <device/irq.h>

#include <string.h>

#include <device/class/spi.h>
#include <device/class/gpio.h>

#include <device/class/usbdev.h>
#include <device/usb/usb.h>

#define MAX3420_FIFO_SIZE 64
#define MAX3420_CURRENT_REVISION 4
#define MAX3420_CHECK_RETRIES 8

#define MAX3420_IO_RST 0
#define MAX3420_IO_IRQ 1
#define MAX3420_IO_GPX 2

#define MAX3420_EP_MASK   0xF

#define MAX3420_EP0_MASK   0x1
#define MAX3420_EP1_MASK   0x2
#define MAX3420_EP2_MASK   0x4
#define MAX3420_EP3_MASK   0x8
#define MAX3420_CFG_MASK   0x10
#define MAX3420_CNT_MASK   0x20
#define MAX3420_DIS_MASK   0x40
#define MAX3420_INIT_MASK  0x80

#define MAX3420_EP0FIFO     (0<<3)
#define MAX3420_EP1OUTFIFO  (1<<3)
#define MAX3420_EP2INFIFO   (2<<3)
#define MAX3420_EP3INFIFO   (3<<3)
#define MAX3420_SUDFIFO     (4<<3)
#define MAX3420_EP0BC       (5<<3)
#define MAX3420_EP1OUTBC    (6<<3)
#define MAX3420_EP2INBC     (7<<3)
#define MAX3420_EP3INBC     (8<<3)
#define MAX3420_EPSTALLS    (9<<3)
#define MAX3420_CLRTOGS     (10<<3)
#define MAX3420_EPIRQ       (11<<3)
#define MAX3420_EPIEN       (12<<3)
#define MAX3420_USBIRQ      (13<<3)
#define MAX3420_USBIEN      (14<<3)
#define MAX3420_USBCTRL     (15<<3)
#define MAX3420_CPUCTRL     (16<<3)
#define MAX3420_PINCTRL     (17<<3)
#define MAX3420_REVISION    (18<<3)
#define MAX3420_FNADDR      (19<<3)
#define MAX3420_IOPINS      (20<<3)
                                  
#define MAX3420_IRQ_IN0      0x1
#define MAX3420_IRQ_OUT0     0x2
#define MAX3420_IRQ_OUT1     0x4
#define MAX3420_IRQ_IN2      0x8
#define MAX3420_IRQ_IN3      0x10
#define MAX3420_IRQ_SETUP    0x20

#define MAX3420_ACKSTAT      0x40
#define MAX3420_STLSTAT      0x20
#define MAX3420_STLEP3IN     0x10
#define MAX3420_STLEP2IN     0x08
#define MAX3420_STLEP1OUT    0x04
#define MAX3420_STLEP0OUT    0x02
#define MAX3420_STLEP0IN     0x01

#define MAX3420_EPIRQ_MASK  0x3F

#define MAX3420_IRQ_RSTDONE 0x80
#define MAX3420_IRQ_VBUS    0x40
#define MAX3420_IRQ_NOVBUS  0x20
#define MAX3420_IRQ_SUSP    0x10

#define MAX3420_USBIRQ_MASK 0xE0

#define MAX3420_EPIN_MSK   0xD
#define MAX3420_EPOUT_MSK  0x3

#define R_TMP0      0
#define R_TMP1      1
#define R_TMP2      2
#define R_TMP3      3
#define R_ARG0      4
#define R_ARG1      5 
#define R_ARG2      6 
#define R_ARG3      7 
#define R_ARG4      8 
#define R_CTX_PV    9
#define R_BUFF      10
#define R_SIZE      11
#define R_TRANSFER  12
#define R_ICOUNT    13
#define R_LINK      14

struct max3420_bytecode_entry_s;
extern struct max3420_bytecode_entry_s max3420_entry_reset;
extern struct max3420_bytecode_entry_s max3420_entry_setup;
extern struct max3420_bytecode_entry_s max3420_entry_data_in_0;
extern struct max3420_bytecode_entry_s max3420_entry_data_out_0;
extern struct max3420_bytecode_entry_s max3420_entry_status;
extern struct max3420_bytecode_entry_s max3420_entry_irq;
extern struct max3420_bytecode_entry_s max3420_entry_connect;
extern struct max3420_bytecode_entry_s max3420_entry_disconnect;
extern struct max3420_bytecode_entry_s max3420_entry_unconfigure;
extern struct max3420_bytecode_entry_s max3420_entry_set_address;
extern struct max3420_bytecode_entry_s max3420_entry_epn;
extern const struct bc_descriptor_s max3420_bytecode;

#define MAX3420_EP_COUNT 3

DRIVER_PV(struct max3420_usbdev_private_s
{
  /* base 500 us time */
  dev_timer_delay_t bt;
  /* On-going transfer */
  struct dev_usbdev_config_s * cfg;
  struct dev_usbdev_rq_s *tr[MAX3420_EP_COUNT + 1];
  uint8_t mps[MAX3420_EP_COUNT + 1];
  /* USB device context */
  struct dev_usbdev_context_s usbdev_ctx;
  /* Endpoint */
  struct usbdev_endpoint_s ep[MAX3420_EP_COUNT];
  /* Device SPI */
  struct device_spi_ctrl_s spi;
  struct dev_irq_src_s src_ep;
  struct dev_spi_ctrl_bytecode_rq_s spi_rq;

  uint16_t icount;
  uint8_t event;
  uint8_t done;
  uint8_t pending;
  uint8_t flags;

  gpio_id_t pin_map[4];
});


