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
*/

#include <hexo/iospace.h>

#include <arch/efm32/irq.h>
#include <arch/efm32/cmu.h>
#include <arch/efm32/gpio.h>
#include <arch/efm32/pin.h>

#include <device/class/iomux.h>
#include <device/resources.h>
#include <device/class/gpio.h>

#ifdef CONFIG_DRIVER_RFPACKET_S2LP

static const uint8_t static_rfcfg_array[] = {
  0x00, 0x96, 0x00, 0x00, 0x42, 0x42, 0x19, 0x03, 0x00, 0x18, 0x38, 0x0a,
  0x00, 0x0c, 0x3d, 0x00, 0x92, 0xa7, 0xa7, 0x03, 0x93, 0x33, 0x06, 0x00,
  0x62, 0x07, 0x01, 0x8a, 0xd0, 0x06, 0x00, 0x05, 0x02, 0x29, 0xa0, 0xca,
};

static const uint8_t static_pkcfg_array[] = {
  0x03, 0x40, 0x19, 0x08, 0x00, 0x2b, 0x40, 0x20, 0x00, 0x01, 0x01, 0x40,
  0x06, 0x00, 0x33, 0x00, 0x00, 0x34, 0x12, 0x05, 0x00, 0x39, 0x40, 0x03,
  0x08, 0x06, 0x00, 0x46, 0x01, 0x00, 0x01, 0x00,
};

DEV_DECLARE_STATIC(s2lp_dev, "rfpacket*", 0, s2lp_drv,
  // spi controller
  DEV_STATIC_RES_DEV_PARAM("spi", "/spi*"),
  // gpio controller
  DEV_STATIC_RES_DEV_PARAM("gpio", "/gpio"),
  // irq
  DEV_STATIC_RES_DEV_PARAM("icu", "/gpio"),
  DEV_STATIC_RES_IRQ(0, EFM32_PC5, DEV_IRQ_SENSE_FALLING_EDGE, 0, 1),
  // gpio
  DEV_STATIC_RES_GPIO("nirq", EFM32_PC5,  1),
  DEV_STATIC_RES_GPIO("sdn",  EFM32_PD6,  1),
  // chip select
  DEV_STATIC_RES_UINT_PARAM("gpio-cs-id", EFM32_PD3),
  // static config
  DEV_STATIC_RES_BLOB_PARAM("static_rf", static_rfcfg_array),
  DEV_STATIC_RES_BLOB_PARAM("static_pk", static_pkcfg_array),
);
#endif

