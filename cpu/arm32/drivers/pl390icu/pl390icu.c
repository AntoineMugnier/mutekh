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

    Copyright (c) 2013 Jeremie Brunel <jeremie.brunel@telecom-paristech.fr>
    Copyright (c) 2013 Alexandre Becoulet <alexandre.becoulet@telecom-paristech.fr>
    Copyright (c) 2013 Institut Telecom / Telecom ParisTech

*/

#include <string.h>
#include <stdio.h>

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <hexo/bit.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/class/icu.h>
#include <device/irq.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include "pl390icu_gicc_regs.h"
#include "pl390icu_gicd_regs.h"

/*
  Sink endpoints numbers are:

    32   to 1024: Shared peripheral interrupts
    1024 to 1039: Private interrupts of cpu0
    1040 to 1055: Private interrupts of cpu1
    ...
    1024+16*cpuid to 1024+16*cpuid+15: Private interrupts of cpu #id
 */

DRIVER_PV(struct pl390_icu_private_s
{
  uintptr_t gicd_addr;
  uintptr_t gicc_addr;

  uint_fast8_t cpu_count; // number of cpu interfaces
  uint_fast8_t reg_count; // number of irq registers count

  uint32_t imp_irq[32];  // mask of implemented irqs

  struct dev_irq_sink_s *sinks; // spi_count entries, then ppi_count * cpu_count
  struct dev_irq_src_s *srcs;
});

static DEV_ICU_GET_SINK(pl390_icu_get_sink)
{
  struct device_s *dev = accessor->dev;
  struct pl390_icu_private_s *pv = dev->drv_pv;

  if (id >= 32 && id < pv->reg_count * 32)
    return (pv->imp_irq[id / 32] & (1 << (id % 32)))
      ? pv->sinks + id - 32 : NULL;          /* spi */

  if (id >= 1024)
    {
      id -= 1024;
      uint_fast8_t cpu = id / 16;
      id %= 16;
      if (cpu >= pv->cpu_count)
        return NULL;
      return (pv->imp_irq[0] & (0x10000 << id)) ?   /* ppi */
        pv->sinks + (pv->reg_count - 1) * 32 + cpu * 16 + id : NULL;
    }

  return NULL;
}

static inline uint_fast8_t
pl390_get_current_cpu(struct pl390_icu_private_s *pv, uint_fast8_t ppi_id)
{
  uint8_t mask = (endian_le32(cpu_mem_read_32(pv->gicd_addr + PL390_GICD_ITARGETSR_ADDR(4 + ppi_id / 4)))
                  >> ((ppi_id % 4) * 8));

  return bit_ctz(mask);
}

static DEV_IRQ_SINK_UPDATE(pl390_icu_sink_update)
{
  struct device_s *dev = sink->base.dev;
  struct pl390_icu_private_s *pv = dev->drv_pv;
  uint_fast8_t sink_id = sink - pv->sinks;
  uint_fast16_t id;

  if (sink_id < (pv->reg_count - 1) * 32) /* spi */
    {
      id = sink_id + 32;
    }
  else                                    /* ppi */
    {
      sink_id -= (pv->reg_count - 1) * 32;
      id = sink_id % 16;

      /* check cpu id */
      if (sink_id / 16 != pl390_get_current_cpu(pv, id))
        return;
    }

  uint_fast8_t mode;

  switch (sense)
    {
    case DEV_IRQ_SENSE_HIGH_LEVEL:
      mode = 0;
      break;

    case DEV_IRQ_SENSE_RISING_EDGE:
      mode = PL390_GICD_ICFR_IRQ_EDGE;
      break;

    case DEV_IRQ_SENSE_NONE:
      cpu_mem_write_32(pv->gicd_addr + PL390_GICD_ICENABLER_ADDR(id / 32),
                       endian_le32(1 << (id % 32)));
    default:
      return;
    }

  cpu_mem_write_32(pv->gicd_addr + PL390_GICD_ISENABLER_ADDR(id / 32),
                   endian_le32(1 << (id % 32)));

  uintptr_t a = pv->gicd_addr + PL390_GICD_ICFR_ADDR(id / 16);
  uint32_t x = cpu_mem_read_32(a);
  PL390_GICD_ICFR_IRQ_SETVAL(id % 16, x, mode);
  cpu_mem_write_32(a, x);
}

#define pl390_icu_link device_icu_dummy_link

static DEV_IRQ_SRC_PROCESS(pl390_icu_source_process)
{
  struct device_s *dev = ep->base.dev;
  struct pl390_icu_private_s *pv = dev->drv_pv;

  while (1)
    {
      uint32_t iar = endian_le32(cpu_mem_read_32(pv->gicc_addr + PL390_GICC_IAR_ADDR));

      uint_fast16_t j = PL390_GICC_IAR_INTID_GET(iar);
      struct dev_irq_sink_s *sink;

      switch (j)
        {
        case 1023:
          return;

        case 32 ... 1022:       /* spi */
          j -= 32;
          sink = pv->sinks + j;
          device_irq_sink_process(sink, 0);
          break;

        case 16 ... 31:         /* ppi */
          j -= 16;
          sink = pv->sinks
            + (pv->reg_count - 1) * 32
            + 16 * pl390_get_current_cpu(pv, j)
            + j;
          device_irq_sink_process(sink, 0);
          break;

        case 0 ... 15:          /* sgi */
          /* handle IPIs here */
          break;
        }

      cpu_mem_write_32(pv->gicc_addr + PL390_GICC_EOIR_ADDR, endian_le32(iar));
    }
}

static const struct dev_enum_ident_s  pl390_icu_ids[] =
{
  DEV_ENUM_FDTNAME_ENTRY("pl390"),
  { 0 }
};

#define pl390_icu_use dev_use_generic

static
void pl390_icu_cpu_init(struct device_s *dev)
{
  struct pl390_icu_private_s  *pv = dev->drv_pv;

  /* level sensitive, N-N model */
  //  cpu_mem_write_32(pv->gicd_addr + PL390_GICD_ICFR_ADDR(0), 0);

  cpu_mem_write_32(pv->gicd_addr + PL390_GICD_ICENABLER_ADDR(0), endian_le32(0xffff0000));
  
  uint_fast16_t i;
  for (i = 0; i < 8; i++)
    {
      cpu_mem_write_32(pv->gicd_addr + PL390_GICD_IPRIORITY_ADDR(i), 0xa0a0a0a0);
    }

  /* enable cpu interface */
  cpu_mem_write_32(pv->gicc_addr + PL390_GICC_PMR_ADDR, endian_le32(0xf0));
  cpu_mem_write_32(pv->gicc_addr + PL390_GICC_CTRL_ADDR, endian_le32(PL390_GICC_CTRL_ENABLE));
}

static DEV_INIT(pl390_icu_init)
{
  struct pl390_icu_private_s  *pv;


  pv = mem_alloc(sizeof (*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;
  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->gicd_addr, NULL) ||
      device_res_get_uint(dev, DEV_RES_MEM, 1, &pv->gicc_addr, NULL))
    goto err_mem;

  /* disable all irqs */
  cpu_mem_write_32(pv->gicd_addr + PL390_GICD_CTRL_ADDR, 0);

  /* find implemented irqs */
  uint32_t typer = endian_le32(cpu_mem_read_32(pv->gicd_addr + PL390_GICD_TYPER_ADDR));

  pv->cpu_count = PL390_GICD_TYPER_CPUNUMBER_GET(typer) + 1;
  uint_fast16_t i, n = PL390_GICD_TYPER_ITLINES_GET(typer) + 1;

  for (i = 0; i < n; i++)
    {
      cpu_mem_write_32(pv->gicd_addr + PL390_GICD_ISENABLER_ADDR(i), 0xffffffff);
      pv->imp_irq[i] = endian_le32(cpu_mem_read_32(pv->gicd_addr + PL390_GICD_ISENABLER_ADDR(i)));
    }

  for (; i < 32; i++)
    pv->imp_irq[i] = 0;

  uint_fast16_t spi_count = 0;
  for (i = 1; i < n; i++)
    spi_count += bit_popc(pv->imp_irq[i]);

  uint_fast16_t ppi_count = bit_popc16(pv->imp_irq[0] >> 16);
  pv->reg_count = n;

  printk("pl390: %u cpu interfaces, %u shared peripheral irqs, %u private irqs per cpu\n",
         pv->cpu_count, spi_count, ppi_count);

  /* init */

  for (i = 8; i < n * 32 / 4; i++)
    {
      cpu_mem_write_32(pv->gicd_addr + PL390_GICD_IPRIORITY_ADDR(i), 0xa0a0a0a0);
      /* FIXME CPU 0 handle all irqs */
      cpu_mem_write_32(pv->gicd_addr + PL390_GICD_ITARGETSR_ADDR(i), 0x01010101);
    }

  for (i = 1; i < n; i++)
    {
      /* level sensitive, N-N model */
      cpu_mem_write_32(pv->gicd_addr + PL390_GICD_ICFR_ADDR(i), 0);
      /* disable irqs */
      cpu_mem_write_32(pv->gicd_addr + PL390_GICD_ICENABLER_ADDR(i), 0xffffffff);
    }

  cpu_mem_write_32(pv->gicd_addr + PL390_GICD_ICENABLER_ADDR(0), 0xffffffff);

#warning FIXME cpu local init
  pl390_icu_cpu_init(dev);

  /* enable distributor */
  cpu_mem_write_32(pv->gicd_addr + PL390_GICD_CTRL_ADDR, endian_le32(PL390_GICD_CTRL_ENABLE));

  /* create irq endpoints */

  pv->srcs = mem_alloc(sizeof(pv->srcs[0]) * pv->cpu_count, (mem_scope_sys));
  if (!pv->srcs)
    goto err_mem;

  /* FIXME don't allocate endpoint for non implemented irqs */
  pv->sinks = mem_alloc(sizeof(pv->sinks[0]) * ((n - 1) * 32 + pv->cpu_count * 16), (mem_scope_sys));
  if (!pv->sinks)
    goto err_mem2;

  device_irq_source_init(dev, pv->srcs, pv->cpu_count,
                         &pl390_icu_source_process);
  if (device_irq_source_link(dev, pv->srcs, pv->cpu_count, -1))
    goto err_mem3;

  device_irq_sink_init(dev, pv->sinks, ((n - 1) * 32 + pv->cpu_count * 16),
                       &pl390_icu_sink_update, DEV_IRQ_SENSE_HIGH_LEVEL | DEV_IRQ_SENSE_RISING_EDGE);

  return 0;

 err_mem3:
  mem_free(pv->sinks);
 err_mem2:
  mem_free(pv->srcs);
 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(pl390_icu_cleanup)
{
  struct pl390_icu_private_s *pv = dev->drv_pv;

  /* disable all irqs */

  if (pv->sinks)
    mem_free(pv->sinks);

  mem_free(pv);
}

DRIVER_DECLARE(pl390_icu_drv, 0, "PL390 ARM generic interrupts controller", pl390_icu,
               DRIVER_ICU_METHODS(pl390_icu));

DRIVER_REGISTER(pl390_icu_drv,
                DEV_ENUM_FDTNAME_ENTRY("pl390"));

