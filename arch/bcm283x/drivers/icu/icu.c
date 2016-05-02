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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2013
    Copyright Jeremie Brunel <jeremie.brunel@telecom-paristech.fr> (c) 2013

*/

#include <string.h>
#include <stdio.h>

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/class/icu.h>
#include <device/irq.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

/*
  0-7: Arm irqs
	0 ARM Timer
	1 ARM Mailbox
	2 ARM Doorbell 0
	3 ARM Doorbell 1
	4 GPU0 halted
	5 GPU1 halted
	6 Illegal access type 1
	7 Illegal access type 0
  8-71: GPU irqs
*/
#define BCM283XICU_MAX_VECTOR	(8+64)

#define BCM283XICU_BAS_PEND 0x200
#define BCM283XICU_PEND1    0x204
#define BCM283XICU_PEND2    0x208
#define BCM283XICU_FIQCTRL  0x20c
#define BCM283XICU_ENA1     0x210
#define BCM283XICU_ENA2     0x214
#define BCM283XICU_BAS_ENA  0x218
#define BCM283XICU_DIS1     0x21c
#define BCM283XICU_DIS2     0x220
#define BCM283XICU_BAS_DIS  0x224

struct bcm283x_icu_private_s
{
  uintptr_t addr;

  struct dev_irq_sink_s sinks[BCM283XICU_MAX_VECTOR];
  struct dev_irq_src_s src;
};

static DEV_ICU_GET_SINK(bcm283x_icu_get_sink)
{
  struct device_s *dev = accessor->dev;
  struct bcm283x_icu_private_s *pv = dev->drv_pv;

  if (id < BCM283XICU_MAX_VECTOR)
    return &pv->sinks[id];
  return NULL;
}

static DEV_IRQ_SINK_UPDATE(bcm283x_icu_sink_update)
{
  struct device_s *dev = sink->base.dev;
  struct bcm283x_icu_private_s *pv = dev->drv_pv;
  uint_fast8_t sink_id = sink - pv->sinks;

  switch (sense)
    {
    case DEV_IRQ_SENSE_NONE:
      switch (sink_id)
        {
        case 0 ... 7:
          cpu_mem_write_32(pv->addr + BCM283XICU_BAS_DIS, endian_le32(1 << sink_id));
          break;
        case 8 ... 39:
          cpu_mem_write_32(pv->addr + BCM283XICU_DIS1, endian_le32(1 << (sink_id - 8)));
          break;
        case 40 ... 71:
          cpu_mem_write_32(pv->addr + BCM283XICU_DIS2, endian_le32(1 << (sink_id - 40)));
          break;
        }
      break;
    case DEV_IRQ_SENSE_HIGH_LEVEL:
      switch (sink_id)
        {
        case 0 ... 7:
          cpu_mem_write_32(pv->addr + BCM283XICU_BAS_ENA, endian_le32(1 << sink_id));
          break;
        case 8 ... 39:
          cpu_mem_write_32(pv->addr + BCM283XICU_ENA1, endian_le32(1 << (sink_id - 8)));
          break;
        case 40 ... 71:
          cpu_mem_write_32(pv->addr + BCM283XICU_ENA2, endian_le32(1 << (sink_id - 40)));
          break;
        }
      break;
    default:
      return;
    }
}

#define bcm283x_icu_link device_icu_dummy_link

static DEV_IRQ_SRC_PROCESS(bcm283x_icu_source_process)
{
  struct device_s *dev = ep->base.dev;
  struct bcm283x_icu_private_s *pv = dev->drv_pv;
  static const uint8_t bcm283x_icu_basic_to_gpu[21] =
    {
      [10] = 7+8,  [11] = 9+8,  [12] = 10+8, [13] = 18+8,
      [14] = 19+8, [15] = 53+8, [16] = 54+8, [17] = 55+8,
      [18] = 56+8, [19] = 57+8, [20] = 62+8
    };

  while (1)
    {
      uint32_t basic = endian_le32(cpu_mem_read_32(pv->addr + BCM283XICU_BAS_PEND));

      uint_fast8_t i;
      if (basic & 0x000000ff)       // arm irqs 0-7
        i = __builtin_ctz(basic);     // clz instruction on ARM cpu

      else if (basic & 0x001ffc00)   // some gpu irqs directly mapped in the basic pending
        i = bcm283x_icu_basic_to_gpu[__builtin_ctz(basic & 0x001ffc00)];

      else if (basic & 0x00000100)       // gpu irqs 0-31
        i = __builtin_ctz(endian_le32(cpu_mem_read_32(pv->addr + BCM283XICU_PEND1))) + 8;

      else if (basic & 0x00000200)       // gpu irqs 32-63
        i = __builtin_ctz(endian_le32(cpu_mem_read_32(pv->addr + BCM283XICU_PEND2))) + 8 + 32;
      else
        break;

      struct dev_irq_sink_s *sink = pv->sinks + i;
      device_irq_sink_process(sink, 0);
    }
}


#define bcm283x_icu_use dev_use_generic

static DEV_INIT(bcm283x_icu_init)
{
  struct bcm283x_icu_private_s  *pv;


  pv = mem_alloc(sizeof (*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;
  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

  /* disable all irqs */
  cpu_mem_write_32(pv->addr + BCM283XICU_BAS_DIS, 0xff);
  cpu_mem_write_32(pv->addr + BCM283XICU_DIS1, 0xffffffff);
  cpu_mem_write_32(pv->addr + BCM283XICU_DIS2, 0xffffffff);

  /* enable pending regs 1 & 2 */
  cpu_mem_write_32(pv->addr + BCM283XICU_BAS_ENA, endian_le32(3 << 8));

  device_irq_source_init(dev, &pv->src, 1, &bcm283x_icu_source_process);
  if (device_irq_source_link(dev, &pv->src, 1, 0))
    goto err_mem;

  device_irq_sink_init(dev, pv->sinks, BCM283XICU_MAX_VECTOR,
                       &bcm283x_icu_sink_update, DEV_IRQ_SENSE_HIGH_LEVEL);

  return 0;

 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(bcm283x_icu_cleanup)
{
  struct bcm283x_icu_private_s *pv = dev->drv_pv;

  /* disable all irqs */
  cpu_mem_write_32(pv->addr + BCM283XICU_BAS_DIS, 0xff);
  cpu_mem_write_32(pv->addr + BCM283XICU_DIS1, 0xffffffff);
  cpu_mem_write_32(pv->addr + BCM283XICU_DIS2, 0xffffffff);

  /* detach bcm283x_icu irq end-points */
  device_irq_source_unlink(dev, &pv->src, 1);

  if (pv->sinks)
    mem_free(pv->sinks);

  mem_free(pv);
}

DRIVER_DECLARE(bcm283x_icu_drv, 0, "BCM283X irq controller", bcm283x_icu,
               DRIVER_ICU_METHODS(bcm283x_icu));

DRIVER_REGISTER(bcm283x_icu_drv,
                DEV_ENUM_FDTNAME_ENTRY("bcm283x_icu"));

