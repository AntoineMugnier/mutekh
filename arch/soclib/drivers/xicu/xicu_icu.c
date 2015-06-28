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

DEV_ICU_GET_SINK(soclib_xicu_icu_get_sink)
{
  struct device_s *dev = accessor->dev;
  struct soclib_xicu_private_s *pv = dev->drv_pv;

  if (id < pv->hwi_count)
    return &pv->sinks[id].sink;
  return NULL;
}

#warning add IRQ load balance policy
static void soclib_xicu_rr_mask(struct soclib_xicu_private_s *pv, struct soclib_xicu_sink_s* xsink)
{
  uint_fast8_t sink_id = xsink - pv->sinks;

#if 1
  /* enable hwi on first affinity source */
  uint_fast8_t first = ffs(xsink->affinity) - 1;
  cpu_mem_write_32(XICU_REG_ADDR(pv->addr, XICU_MSK_HWI_ENABLE, first), endian_le32(1 << sink_id));
#else
  /* round robin stuff */

  if (!xsink->counter--)
    {
      xsink->counter = XICU_RR_COUNT;

      cpu_mem_write_32(XICU_REG_ADDR(pv->addr, XICU_MSK_HWI_DISABLE, xsink->current), endian_le32(1 << sink_id));

      do {
        xsink->current = (xsink->current + 1) % pv->irq_count;
      } while (!(xsink->affinity & (1 << xsink->current)));

      cpu_mem_write_32(XICU_REG_ADDR(pv->addr, XICU_MSK_HWI_ENABLE, xsink->current), endian_le32(1 << sink_id));
    }
#endif
}

DEV_IRQ_SINK_UPDATE(soclib_xicu_icu_sink_update)
{
  struct device_s *dev = sink->base.dev;
  struct soclib_xicu_private_s *pv = dev->drv_pv;
  struct soclib_xicu_sink_s *xsink = (struct soclib_xicu_sink_s*)sink;
  uint_fast8_t sink_id = xsink - pv->sinks;

  switch (sense)
    {
    case DEV_IRQ_SENSE_NONE:
      cpu_mem_write_32(XICU_REG_ADDR(pv->addr, XICU_MSK_HWI_DISABLE, xsink->current), endian_le32(1 << sink_id));
      pv->sinks[sink_id].affinity = 0;
      break;
    case DEV_IRQ_SENSE_HIGH_LEVEL:
      xsink->current = 0;
      xsink->counter = 0;
      soclib_xicu_rr_mask(pv, xsink);
    default:
      break;
    }
}

DEV_ICU_LINK(soclib_xicu_icu_link)
{
  if (!route_mask || *bypass)
    return 0;

  struct device_s *dev = accessor->dev;
  struct soclib_xicu_private_s *pv = dev->drv_pv;
  struct soclib_xicu_sink_s *xsink = (struct soclib_xicu_sink_s*)sink;
  uint_fast8_t sink_id = xsink - pv->sinks;

  if (xsink->affinity && *route_mask != xsink->affinity)
    {
      xsink->affinity |= *route_mask;
      printk("xicu %p: shared interrupt on sink %u will change affinity\n", dev, sink_id);
    }
  else
    {
      xsink->affinity = *route_mask;
    }

  return 0;
}

DEV_IRQ_SRC_PROCESS(soclib_xicu_source_process)
{
  struct dev_irq_src_s  *src = ep;
  struct device_s *dev = ep->base.dev;
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
        struct dev_irq_sink_s *sink = &xsink->sink;
        device_irq_sink_process(sink, 0);
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

