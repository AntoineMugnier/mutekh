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

    Copyright (c) 2012 Alexandre Becoulet <alexandre.becoulet@telecom-paristech.fr>
    Copyright (c) 2012 Institut Telecom / Telecom ParisTech

*/

#include "xicu_private.h"

#include <string.h>
#include <stdio.h>

#include <hexo/iospace.h>
#include <hexo/endian.h>

#include <device/class/icu.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#define XICU_RR_COUNT 5

static DEV_ICU_GET_ENDPOINT(soclib_xicu_icu_get_endpoint)
{
  struct device_s *dev = accessor->dev;
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

static DEV_ICU_ENABLE_IRQ(soclib_xicu_icu_enable_irq)
{
  struct device_s *dev = accessor->dev;
  struct soclib_xicu_private_s *pv = dev->drv_pv;
  struct soclib_xicu_sink_s *xsink = (struct soclib_xicu_sink_s*)sink;
  uint_fast8_t i, icu_in_id = xsink - pv->sinks;

  if (irq_id > 0)
    {
      printk("xicu %p: single wire IRQ must use 0 as logical IRQ id for %p device\n", dev, dev_ep->dev);
      return 0;
    }

  // find routes through all source end-points which will relay this interrupt
  uint32_t     affinity = 0;
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

static DEV_ICU_DISABLE_IRQ(soclib_xicu_icu_disable_irq)
{
  struct device_s *dev = accessor->dev;
  struct soclib_xicu_private_s *pv = dev->drv_pv;
  struct soclib_xicu_sink_s *xsink = (struct soclib_xicu_sink_s*)sink;
  uint_fast8_t icu_in_id = xsink - pv->sinks;

  // disable relaying of this sink
  cpu_mem_write_32(XICU_REG_ADDR(pv->addr, XICU_MSK_HWI_DISABLE, xsink->current), endian_le32(1 << icu_in_id));

  pv->sinks[icu_in_id].affinity = 0;
}

DEV_IRQ_EP_PROCESS(soclib_xicu_source_process)
{
  struct dev_irq_ep_s  *src = ep;
  struct device_s *dev = ep->dev;
  struct soclib_xicu_private_s *pv = dev->drv_pv;
  uint_fast8_t out = src - pv->srcs;

  assert(out < pv->irq_count);

  bool_t done;
  do {
    uint32_t prio = endian_le32(cpu_mem_read_32(XICU_REG_ADDR(pv->addr, XICU_PRIO, out)));
    done = 1;

    if (XICU_PRIO_HAS_HWI(prio))
      {
        done = 0;
        struct soclib_xicu_sink_s *xsink = pv->sinks + XICU_PRIO_HWI(prio);
        struct dev_irq_ep_s *sink = &xsink->sink;
        sink->process(sink, id);
        soclib_xicu_rr_mask(pv, xsink);
      }

#ifdef CONFIG_DRIVER_SOCLIB_XICU_TIMER
    if (XICU_PRIO_HAS_PTI(prio))
      {
        done = 0;
        uint_fast8_t i = XICU_PRIO_PTI(prio);
        soclib_xicu_pti_irq_process(dev, i);
        cpu_mem_read_32(XICU_REG_ADDR(pv->addr, XICU_PTI_ACK, i));
      }
#endif

#if defined(CONFIG_ARCH_SMP) || defined(CONFIG_HEXO_IPI)
    if (XICU_PRIO_HAS_WTI(prio))
      {
        done = 0;
        uint_fast8_t i = XICU_PRIO_HWI(prio);
        cpu_mem_read_32(XICU_REG_ADDR(pv->addr, XICU_WTI_REG, i));  /* ack */
      }
#endif

  } while (!done);
}

const struct driver_icu_s  soclib_xicu_icu_drv =
{
  .class_         = DRIVER_CLASS_ICU,
  .f_get_endpoint = soclib_xicu_icu_get_endpoint,
  .f_disable_irq  = soclib_xicu_icu_disable_irq,
  .f_enable_irq   = soclib_xicu_icu_enable_irq,
};

