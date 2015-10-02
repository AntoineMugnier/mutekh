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
#define BCM2835ICU_MAX_VECTOR	(8+64)

#define BCM2835ICU_BAS_PEND 0x200
#define BCM2835ICU_PEND1    0x204
#define BCM2835ICU_PEND2    0x208
#define BCM2835ICU_FIQCTRL  0x20c
#define BCM2835ICU_ENA1     0x210
#define BCM2835ICU_ENA2     0x214
#define BCM2835ICU_BAS_ENA  0x218
#define BCM2835ICU_DIS1     0x21c
#define BCM2835ICU_DIS2     0x220
#define BCM2835ICU_BAS_DIS  0x224

struct bcm2835_icu_private_s
{
  uintptr_t addr;

  struct dev_irq_sink_s sinks[BCM2835ICU_MAX_VECTOR];
  struct dev_irq_src_s src;
};

static DEV_ICU_GET_SINK(bcm2835_icu_get_sink)
{
  struct device_s *dev = accessor->dev;
  struct bcm2835_icu_private_s *pv = dev->drv_pv;

  if (id < BCM2835ICU_MAX_VECTOR)
    return &pv->sinks[id];
  return NULL;
}

static DEV_IRQ_SINK_UPDATE(bcm2835_icu_sink_update)
{
  struct device_s *dev = sink->base.dev;
  struct bcm2835_icu_private_s *pv = dev->drv_pv;
  uint_fast8_t sink_id = sink - pv->sinks;

  switch (sense)
    {
    case DEV_IRQ_SENSE_NONE:
      switch (sink_id)
        {
        case 0 ... 7:
          cpu_mem_write_32(pv->addr + BCM2835ICU_BAS_DIS, endian_le32(1 << sink_id));
          break;
        case 8 ... 39:
          cpu_mem_write_32(pv->addr + BCM2835ICU_DIS1, endian_le32(1 << (sink_id - 8)));
          break;
        case 40 ... 71:
          cpu_mem_write_32(pv->addr + BCM2835ICU_DIS2, endian_le32(1 << (sink_id - 40)));
          break;
        }
      break;
    case DEV_IRQ_SENSE_HIGH_LEVEL:
      switch (sink_id)
        {
        case 0 ... 7:
          cpu_mem_write_32(pv->addr + BCM2835ICU_BAS_ENA, endian_le32(1 << sink_id));
          break;
        case 8 ... 39:
          cpu_mem_write_32(pv->addr + BCM2835ICU_ENA1, endian_le32(1 << (sink_id - 8)));
          break;
        case 40 ... 71:
          cpu_mem_write_32(pv->addr + BCM2835ICU_ENA2, endian_le32(1 << (sink_id - 40)));
          break;
        }
      break;
    default:
      return;
    }
}

#define bcm2835_icu_link device_icu_dummy_link

static DEV_IRQ_SRC_PROCESS(bcm2835_icu_source_process)
{
  struct device_s *dev = ep->base.dev;
  struct bcm2835_icu_private_s *pv = dev->drv_pv;
  static const uint8_t bcm2835_icu_basic_to_gpu[21] =
    {
      [10] = 7+8,  [11] = 9+8,  [12] = 10+8, [13] = 18+8,
      [14] = 19+8, [15] = 53+8, [16] = 54+8, [17] = 55+8,
      [18] = 56+8, [19] = 57+8, [20] = 62+8
    };

  while (1)
    {
      uint32_t basic = endian_le32(cpu_mem_read_32(pv->addr + BCM2835ICU_BAS_PEND));

      uint_fast8_t i;
      if (basic & 0x000000ff)       // arm irqs 0-7
        i = __builtin_ctz(basic);     // clz instruction on ARM cpu

      else if (basic & 0x001ffc00)   // some gpu irqs directly mapped in the basic pending
        i = bcm2835_icu_basic_to_gpu[__builtin_ctz(basic & 0x001ffc00)];

      else if (basic & 0x00000100)       // gpu irqs 0-31
        i = __builtin_ctz(endian_le32(cpu_mem_read_32(pv->addr + BCM2835ICU_PEND1))) + 8;

      else if (basic & 0x00000200)       // gpu irqs 32-63
        i = __builtin_ctz(endian_le32(cpu_mem_read_32(pv->addr + BCM2835ICU_PEND2))) + 8 + 32;
      else
        break;

      struct dev_irq_sink_s *sink = pv->sinks + i;
      device_irq_sink_process(sink, 0);
    }
}

static DEV_INIT(bcm2835_icu_init);
static DEV_CLEANUP(bcm2835_icu_cleanup);

#define bcm2835_icu_use dev_use_generic

DRIVER_DECLARE(bcm2835_icu_drv, 0, "BCM2835 irq controller", bcm2835_icu,
               DRIVER_ICU_METHODS(bcm2835_icu));

DRIVER_REGISTER(bcm2835_icu_drv,
                DEV_ENUM_FDTNAME_ENTRY("bcm2835_icu"));

DEV_INIT(bcm2835_icu_init)
{
  struct bcm2835_icu_private_s  *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  pv = mem_alloc(sizeof (*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;
  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

  /* disable all irqs */
  cpu_mem_write_32(pv->addr + BCM2835ICU_BAS_DIS, 0xff);
  cpu_mem_write_32(pv->addr + BCM2835ICU_DIS1, 0xffffffff);
  cpu_mem_write_32(pv->addr + BCM2835ICU_DIS2, 0xffffffff);

  /* enable pending regs 1 & 2 */
  cpu_mem_write_32(pv->addr + BCM2835ICU_BAS_ENA, endian_le32(3 << 8));

  device_irq_source_init(dev, &pv->src, 1, &bcm2835_icu_source_process);
  if (device_irq_source_link(dev, &pv->src, 1, 0))
    goto err_mem;

  device_irq_sink_init(dev, pv->sinks, BCM2835ICU_MAX_VECTOR,
                       &bcm2835_icu_sink_update, DEV_IRQ_SENSE_HIGH_LEVEL);

  dev->drv = &bcm2835_icu_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;
  return 0;

 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(bcm2835_icu_cleanup)
{
  struct bcm2835_icu_private_s *pv = dev->drv_pv;

  /* disable all irqs */
  cpu_mem_write_32(pv->addr + BCM2835ICU_BAS_DIS, 0xff);
  cpu_mem_write_32(pv->addr + BCM2835ICU_DIS1, 0xffffffff);
  cpu_mem_write_32(pv->addr + BCM2835ICU_DIS2, 0xffffffff);

  /* detach bcm2835_icu irq end-points */
  device_irq_sink_unlink(dev, pv->sinks, BCM2835ICU_MAX_VECTOR);
  device_irq_source_unlink(dev, &pv->src, 1);

  if (pv->sinks)
    mem_free(pv->sinks);

  mem_free(pv);
}

