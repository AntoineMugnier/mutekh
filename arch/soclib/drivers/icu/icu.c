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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2012

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

#define ICU_SOCLIB_MAX_VECTOR	32

#define ICU_SOCLIB_REG_ISR	0
#define ICU_SOCLIB_REG_IER_RO	4
#define ICU_SOCLIB_REG_IER_SET	8
#define ICU_SOCLIB_REG_IER_CLR	12
#define ICU_SOCLIB_REG_VECTOR	16

struct soclib_icu_private_s
{
  uintptr_t addr;
  uintptr_t nirq;

  struct dev_irq_sink_s *sinks;
  struct dev_irq_src_s src;
};

static DEV_ICU_GET_SINK(soclib_icu_icu_get_sink)
{
  struct device_s *dev = accessor->dev;
  struct soclib_icu_private_s *pv = dev->drv_pv;

  if (id < pv->nirq)
    return pv->sinks + id;
  return NULL;
}

#define soclib_icu_icu_link device_icu_dummy_link

static DEV_IRQ_SINK_UPDATE(soclib_icu_icu_sink_update)
{
  struct device_s *dev = sink->base.dev;
  struct soclib_icu_private_s *pv = dev->drv_pv;
  uint_fast8_t sink_id = sink - pv->sinks;

  switch (sense)
    {
    case DEV_IRQ_SENSE_NONE:
      cpu_mem_write_32(pv->addr + ICU_SOCLIB_REG_IER_CLR, endian_le32(1 << sink_id));
      break;
    case DEV_IRQ_SENSE_HIGH_LEVEL:
      cpu_mem_write_32(pv->addr + ICU_SOCLIB_REG_IER_SET, endian_le32(1 << sink_id));
    default:
      break;
    }
}

static DEV_IRQ_SRC_PROCESS(soclib_icu_source_process)
{
  struct device_s *dev = ep->base.dev;
  struct soclib_icu_private_s *pv = dev->drv_pv;

  while (1)
    {
      uint32_t prio = endian_le32(cpu_mem_read_32(pv->addr + ICU_SOCLIB_REG_VECTOR));

      if (prio > pv->nirq)
        break;

      struct dev_irq_sink_s *sink = pv->sinks + prio;
      device_irq_sink_process(sink, 0);
    }
}

static DEV_INIT(soclib_icu_init);
static DEV_CLEANUP(soclib_icu_cleanup);

#define soclib_icu_use dev_use_generic

DRIVER_DECLARE(soclib_icu_drv, 0, "Soclib Icu", soclib_icu,
               DRIVER_ICU_METHODS(soclib_icu_icu));

DRIVER_REGISTER(soclib_icu_drv,
                DEV_ENUM_FDTNAME_ENTRY("soclib:icu"));

static DEV_INIT(soclib_icu_init)
{
  struct soclib_icu_private_s  *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  pv = mem_alloc(sizeof (*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;
  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

  device_get_param_uint_default(dev, "nirq", &pv->nirq, ICU_SOCLIB_MAX_VECTOR);

  device_irq_source_init(dev, &pv->src, 1, &soclib_icu_source_process);
  if (device_irq_source_link(dev, &pv->src, 1, 0))
    goto err_mem;

  /* init soclib_icu irq sink end-points */
  pv->sinks = mem_alloc(sizeof(pv->sinks[0]) * pv->nirq, (mem_scope_sys));
  if (!pv->sinks)
    goto err_unlink;

  device_irq_sink_init(dev, pv->sinks, pv->nirq,
                       &soclib_icu_icu_sink_update,
                       DEV_IRQ_SENSE_HIGH_LEVEL);

  dev->drv = &soclib_icu_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;
  return 0;

 err_unlink:
  device_irq_source_unlink(dev, &pv->src, 1);
 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(soclib_icu_cleanup)
{
  struct soclib_icu_private_s *pv = dev->drv_pv;

  /* disable all irqs */
  cpu_mem_write_32(pv->addr + ICU_SOCLIB_REG_IER_CLR, 0xffffffff);

  /* detach soclib_icu irq end-points */
  device_irq_source_unlink(dev, &pv->src, 1);

  if (pv->sinks)
    mem_free(pv->sinks);

  mem_free(pv);

  return 0;
}

