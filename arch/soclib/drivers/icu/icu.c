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

  struct dev_irq_ep_s *sinks;
  struct dev_irq_ep_s src;
};

static DEV_ICU_GET_ENDPOINT(soclib_icu_icu_get_endpoint)
{
  struct device_s *dev = accessor->dev;
  struct soclib_icu_private_s *pv = dev->drv_pv;

  switch (type)
    {
    case DEV_IRQ_EP_SINK:
      if (id < pv->nirq)
        return pv->sinks + id;
      return NULL;

    case DEV_IRQ_EP_SOURCE:
      if (id < 1)
        return &pv->src;

    default:
      return NULL;
    }
}

static DEV_ICU_ENABLE_IRQ(soclib_icu_icu_enable_irq)
{
  struct device_s *dev = accessor->dev;
  struct soclib_icu_private_s *pv = dev->drv_pv;
  uint_fast8_t icu_in_id = sink - pv->sinks;

  if (irq_id > 0)
    {
      printk("icu %p: single wire IRQ must use 0 as logical IRQ id for %p device\n", dev, dev_ep->dev);
      return 0;
    }

  if (!device_icu_irq_enable(&pv->src, 0, NULL, dev_ep))
    {
      printk("icu: source end-point can not relay interrupt for %p device\n", dev_ep->dev);
      return 0;
    }

  cpu_mem_write_32(pv->addr + ICU_SOCLIB_REG_IER_SET, endian_le32(1 << icu_in_id));

  return 1;
}

static DEV_ICU_DISABLE_IRQ(soclib_icu_icu_disable_irq)
{
  struct device_s *dev = accessor->dev;
  struct soclib_icu_private_s *pv = dev->drv_pv;
  uint_fast8_t icu_in_id = sink - pv->sinks;

  cpu_mem_write_32(pv->addr + ICU_SOCLIB_REG_IER_CLR, endian_le32(1 << icu_in_id));
}

static DEV_IRQ_EP_PROCESS(soclib_icu_source_process)
{
  struct device_s *dev = ep->dev;
  struct soclib_icu_private_s *pv = dev->drv_pv;

  while (1)
    {
      uint32_t prio = endian_le32(cpu_mem_read_32(pv->addr + ICU_SOCLIB_REG_VECTOR));

      if (prio > pv->nirq)
        break;

      struct dev_irq_ep_s *sink = pv->sinks + prio;
      sink->process(sink, id);
    }
}

const struct driver_icu_s  soclib_icu_icu_drv =
{
  .class_         = DRIVER_CLASS_ICU,
  .f_get_endpoint = soclib_icu_icu_get_endpoint,
  .f_enable_irq   = soclib_icu_icu_enable_irq,
  .f_disable_irq  = soclib_icu_icu_disable_irq,
};

static const struct dev_enum_ident_s  soclib_icu_ids[] =
{
  DEV_ENUM_FDTNAME_ENTRY("soclib:icu"),
  { 0 }
};

static DEV_INIT(soclib_icu_init);
static DEV_CLEANUP(soclib_icu_cleanup);

const struct driver_s  soclib_icu_drv =
{
  .desc           = "Soclib VciIcu",
  .id_table       = soclib_icu_ids,

  .f_init         = soclib_icu_init,
  .f_cleanup      = soclib_icu_cleanup,

  .classes        = {
    &soclib_icu_icu_drv,
    0
  }
};

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

  device_irq_source_init(dev, &pv->src, 1,
                         &soclib_icu_source_process, DEV_IRQ_SENSE_HIGH_LEVEL);
  if (device_irq_source_link(dev, &pv->src, 1, 0))
    goto err_mem;

  /* init soclib_icu irq sink end-points */
  pv->sinks = mem_alloc(sizeof(pv->sinks[0]) * pv->nirq, (mem_scope_sys));
  if (!pv->sinks)
    goto err_unlink;

  device_irq_sink_init(dev, pv->sinks, pv->nirq, DEV_IRQ_SENSE_HIGH_LEVEL);

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
  device_irq_sink_unlink(dev, pv->sinks, pv->nirq);
  device_irq_source_unlink(dev, &pv->src, 1);

  if (pv->sinks)
    mem_free(pv->sinks);

  mem_free(pv);
}

REGISTER_DRIVER(soclib_icu_drv);

