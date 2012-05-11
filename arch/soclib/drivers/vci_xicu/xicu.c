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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/

#include "xicu_private.h"

#include <string.h>
#include <stdio.h>

#include <hexo/types.h>
#include <hexo/iospace.h>
#include <hexo/endian.h>

#include <device/device.h>
#include <device/driver.h>
#include <device/class/icu.h>
#include <device/irq.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#define XICU_RR_COUNT 5

struct soclib_xicu_sink_s
{
  struct dev_irq_ep_s sink;
  uint32_t     affinity;
  uint_fast8_t current;
  uint_fast8_t counter;
};

struct soclib_xicu_private_s
{
  uintptr_t addr;

#ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_TIMER
  uintptr_t pti_count;
#endif

#ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_ICU
  uintptr_t hwi_count;
  struct soclib_xicu_sink_s *sinks;

  uintptr_t irq_count;
  struct dev_irq_ep_s *srcs;

# ifdef CONFIG_HEXO_IPI
  uintptr_t wti_count;
# endif
#endif

};

/************************************************************************
        Interrupts controller driver part
************************************************************************/

#ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_ICU

static DEVICU_GET_ENDPOINT(soclib_xicu_icu_get_endpoint)
{
  struct device_s *dev = idev->dev;
  struct soclib_xicu_private_s *pv = dev->drv_pv;

  switch (type)
    {
    case DEV_IRQ_EP_SINK:
      if (id < pv->hwi_count)
        return &pv->sinks[id].sink;
      return NULL;

    case DEV_IRQ_EP_SOURCE:
      if (id < pv->irq_count)
        return pv->srcs + id;

    default:
      return NULL;
    }
}

#warning add IRQ load balance policy
static void soclib_xicu_rr_mask(struct soclib_xicu_private_s *pv, struct soclib_xicu_sink_s* xsink)
{
  uint_fast8_t icu_in_id = xsink - pv->sinks;

#if 1
  /* enable hwi on first affinity source */
  uint_fast8_t first = ffs(xsink->affinity) - 1;
  cpu_mem_write_32(XICU_REG_ADDR(pv->addr, XICU_MSK_HWI_ENABLE, first), endian_le32(1 << icu_in_id));
#else
  /* round robin stuff */

  if (!xsink->counter--)
    {
      xsink->counter = XICU_RR_COUNT;

      cpu_mem_write_32(XICU_REG_ADDR(pv->addr, XICU_MSK_HWI_DISABLE, xsink->current), endian_le32(1 << icu_in_id));

      do {
        xsink->current = (xsink->current + 1) % pv->irq_count;
      } while (!(xsink->affinity & (1 << xsink->current)));

      cpu_mem_write_32(XICU_REG_ADDR(pv->addr, XICU_MSK_HWI_ENABLE, xsink->current), endian_le32(1 << icu_in_id));
    }
#endif
}

static DEVICU_ENABLE_IRQ(soclib_xicu_icu_enable_irq)
{
  struct device_s *dev = idev->dev;
  struct soclib_xicu_private_s *pv = dev->drv_pv;
  struct soclib_xicu_sink_s *xsink = (struct soclib_xicu_sink_s*)sink;
  uint_fast8_t icu_in_id = xsink - pv->sinks;

  if (irq_id > 0)
    {
      printk("xicu %p: single wire IRQ must use 0 as logical IRQ id for %p device\n", dev, dev_ep->dev);
      return 0;
    }

  uint32_t affinity = 0;
  uint_fast8_t i;

  // find routes through all source end-points which will relay this interrupt
  for (i = 0; i < pv->irq_count; i++)
    {
      if (device_icu_irq_enable(pv->srcs + i, 0, NULL, dev_ep))
        affinity |= (1 << i);
    }

  if (!affinity)
    {
      printk("xicu %p: found no source end-point which can relay interrupt for %p device\n", dev, dev_ep->dev);
      return 0;
    }
  else if (xsink->affinity && affinity != xsink->affinity)
    { 
      xsink->affinity |= affinity;
      printk("xicu %p: shared interrupt on sink %u will change affinity\n", dev, icu_in_id);
    }
  else
    {
      xsink->affinity = affinity;
      xsink->current = 0;
      xsink->counter = 0;
      soclib_xicu_rr_mask(pv, xsink);
    }

  return 1;
}

static DEVICU_DISABLE_IRQ(soclib_xicu_icu_disable_irq)
{
  struct device_s *dev = idev->dev;
  struct soclib_xicu_private_s *pv = dev->drv_pv;
  struct soclib_xicu_sink_s *xsink = (struct soclib_xicu_sink_s*)sink;
  uint_fast8_t icu_in_id = xsink - pv->sinks;

  // disable relaying of this sink
  cpu_mem_write_32(XICU_REG_ADDR(pv->addr, XICU_MSK_HWI_DISABLE, xsink->current), endian_le32(1 << icu_in_id));

  pv->sinks[icu_in_id].affinity = 0;
}

static DEV_IRQ_EP_PROCESS(soclib_xicu_source_process)
{
  struct dev_irq_ep_s  *src = ep;
  struct device_s *dev = ep->dev;
  struct soclib_xicu_private_s *pv = dev->drv_pv;
  uint_fast8_t out = src - pv->srcs;

  assert(out < pv->irq_count);

  while (1)
    {
      uint32_t prio = endian_le32(cpu_mem_read_32(XICU_REG_ADDR(pv->addr, XICU_PRIO, out)));

      if (!XICU_PRIO_HAS_HWI(prio))
        break;

      struct soclib_xicu_sink_s *xsink = pv->sinks + XICU_PRIO_HWI(prio);
      struct dev_irq_ep_s *sink = &xsink->sink;
      sink->process(sink, id);
      soclib_xicu_rr_mask(pv, xsink);
    }
}

const struct driver_icu_s  soclib_xicu_icu_drv =
{
  .class_         = DRIVER_CLASS_ICU,
  .f_get_endpoint = soclib_xicu_icu_get_endpoint,
  .f_disable_irq  = soclib_xicu_icu_disable_irq,
  .f_enable_irq   = soclib_xicu_icu_enable_irq,
};

#endif

/************************************************************************
        Timer driver part
************************************************************************/

#ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_TIMER
# warning FIXME implement XICU timer driver
#endif

/************************************************************************/

static const struct devenum_ident_s  soclib_xicu_ids[] =
{
  DEVENUM_FDTNAME_ENTRY("soclib:vci_xicu"),
  { 0 }
};

static DEV_INIT(soclib_xicu_init);
static DEV_CLEANUP(soclib_xicu_cleanup);

const struct driver_s  soclib_xicu_drv =
{
  .desc           = "Soclib VciXicu",
  .id_table       = soclib_xicu_ids,

  .f_init         = soclib_xicu_init,
  .f_cleanup      = soclib_xicu_cleanup,

  .classes        = {
#ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_ICU
    &soclib_xicu_icu_drv,
#endif
    0
  }
};

static DEV_INIT(soclib_xicu_init)
{
  struct soclib_xicu_private_s  *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  pv = mem_alloc(sizeof (*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;
  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

#ifdef CONFIG_DRIVER_SOCLIB_XICU_TIMER
  device_get_param_uint_default(dev, "pti-count", &pv->pti_count, 0);
#endif

#ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_ICU

  device_get_param_uint_default(dev, "irq-count", &pv->irq_count, 1);
  if (pv->irq_count)
    {
      /* init soclib_xicu irq source end-points */
      pv->srcs = mem_alloc(sizeof(pv->srcs[0]) * pv->irq_count, (mem_scope_sys));
      if (!pv->srcs)
        goto err_mem;

      device_irq_source_init(dev, pv->srcs, pv->irq_count, &soclib_xicu_source_process);

      if (device_irq_source_link(dev, pv->srcs, pv->irq_count, 0))
        goto err_mem2;
    }

  device_get_param_uint_default(dev, "hwi-count", &pv->hwi_count, 0);
  if (pv->hwi_count)
    {
      /* init soclib_xicu irq sink end-points */
      pv->sinks = mem_alloc(sizeof(pv->sinks[0]) * pv->hwi_count, (mem_scope_sys));
      if (!pv->sinks)
        goto err_unlink;

      uint_fast8_t i;
      for (i = 0; i < pv->hwi_count; i++)
        {
          device_irq_sink_init(dev, &pv->sinks[i].sink, 1);
          pv->sinks[i].affinity = 0;
        }
    }

# ifdef CONFIG_HEXO_IPI
  device_get_param_uint_default(dev, "wti-count", &pv->wti_count, 0);
# endif

#endif

  dev->drv = &soclib_xicu_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;
  return 0;

#ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_ICU
 err_unlink:
  device_irq_source_unlink(dev, pv->srcs, pv->irq_count);
 err_mem2:
  if (pv->srcs)
    mem_free(pv->sinks);
#endif
 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(soclib_xicu_cleanup)
{
  struct soclib_xicu_private_s *pv = dev->drv_pv;

#ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_ICU
  /* detach soclib_xicu irq end-points */
  uint_fast8_t i;
  for (i = 0; i < pv->hwi_count; i++)
    device_irq_sink_unlink(dev, &pv->sinks[i].sink, 1);
  device_irq_source_unlink(dev, pv->srcs, pv->irq_count);

  if (pv->srcs)
    mem_free(pv->srcs);
  if (pv->sinks)
    mem_free(pv->sinks);
#endif

  mem_free(pv);
}

REGISTER_DRIVER(soclib_xicu_drv);

