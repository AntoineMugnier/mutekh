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

#include <hexo/types.h>
#include <hexo/iospace.h>
#include <hexo/endian.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/class/icu.h>
#include <device/class/timer.h>
#include <device/irq.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

static DEV_USE(soclib_xicu_use)
{
  struct device_accessor_s *accessor = param;

  switch (op)
    {
    case DEV_USE_GET_ACCESSOR:
      switch (accessor->api->class_)
        {
#ifdef CONFIG_DRIVER_SOCLIB_XICU_ICU
        case DRIVER_CLASS_ICU:
          if (accessor->number > 0)
            return -ENOTSUP;
          break;
#endif
#ifdef CONFIG_DRIVER_SOCLIB_XICU_TIMER
        case DRIVER_CLASS_TIMER: {
          struct soclib_xicu_private_s  *pv = accessor->dev->drv_pv;
          if (accessor->number / 2 >= pv->pti_count)
            return -ENOTSUP;
          break;
        }
#endif
        default:
          UNREACHABLE();
        }
    case DEV_USE_PUT_ACCESSOR:
      return 0;
    case DEV_USE_START:
    case DEV_USE_STOP:
      switch (accessor->api->class_)
        {
#ifdef CONFIG_DRIVER_SOCLIB_XICU_TIMER
        case DRIVER_CLASS_TIMER:
          return soclib_xicu_timer_use(accessor, op);
#endif
        default:
          return 0;
        }
    default:
      return -ENOTSUP;
    }
}

static DEV_INIT(soclib_xicu_init);
static DEV_CLEANUP(soclib_xicu_cleanup);

DRIVER_DECLARE(soclib_xicu_drv, 0, "Soclib Xicu", soclib_xicu
#ifdef CONFIG_DRIVER_SOCLIB_XICU_ICU
               , DRIVER_ICU_METHODS(soclib_xicu_icu)
#endif
#ifdef CONFIG_DRIVER_SOCLIB_XICU_TIMER
               , DRIVER_TIMER_METHODS(soclib_xicu_timer)
#endif
               );

DRIVER_REGISTER(soclib_xicu_drv,
                DEV_ENUM_FDTNAME_ENTRY("soclib:xicu"));

static DEV_INIT(soclib_xicu_init)
{
  struct soclib_xicu_private_s  *pv;
  uint_fast8_t i;

#ifdef CONFIG_DRIVER_SOCLIB_XICU_TIMER
  uintptr_t pti_count = 0;
  device_get_param_uint_default(dev, "pti-count", &pti_count, 0);

# ifdef CONFIG_DRIVER_SOCLIB_XICU_ICU
  dev_timer_res_t resolution = SOCLIB_XICU_PTI_DEFAULT_PERIOD;
  device_get_param_uint(dev, "period", &resolution);
  if (resolution < SOCLIB_XICU_PTI_MIN_PERIOD)
    resolution = SOCLIB_XICU_PTI_MIN_PERIOD;
# endif
#endif

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  pv = mem_alloc(sizeof (*pv)
#ifdef CONFIG_DRIVER_SOCLIB_XICU_TIMER
                 + pti_count * sizeof(struct soclib_xicu_pti_s)
#endif
                 , (mem_scope_sys));
  if (!pv)
    return -ENOMEM;
  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

#ifdef CONFIG_DRIVER_SOCLIB_XICU_ICU

  device_get_param_uint_default(dev, "irq-count", &pv->irq_count, 1);
  if (pv->irq_count)
    {
      /* init soclib_xicu irq source end-points */
      pv->srcs = mem_alloc(sizeof(pv->srcs[0]) * pv->irq_count, (mem_scope_sys));
      if (!pv->srcs)
        goto err_mem;

      device_irq_source_init(dev, pv->srcs, pv->irq_count,
                             &soclib_xicu_source_process);

# ifdef CONFIG_DRIVER_SOCLIB_XICU_TIMER
      if (device_irq_source_link(dev, pv->srcs, pv->irq_count, -1))
        goto err_mem2;
# else
      if (device_irq_source_link(dev, pv->srcs, pv->irq_count, 0))
        goto err_mem2;
# endif
    }

  device_get_param_uint_default(dev, "hwi-count", &pv->hwi_count, 0);
  if (pv->hwi_count)
    {
      /* init soclib_xicu irq sink end-points */
      pv->sinks = mem_alloc(sizeof(pv->sinks[0]) * pv->hwi_count, (mem_scope_sys));
      if (!pv->sinks)
        goto err_unlink;

      for (i = 0; i < pv->hwi_count; i++)
        {
          device_irq_sink_init(dev, &pv->sinks[i].sink, 1,
                               &soclib_xicu_icu_sink_update,
                               DEV_IRQ_SENSE_HIGH_LEVEL);
          pv->sinks[i].affinity = 0;
        }
    }

# ifdef CONFIG_HEXO_IPI
  device_get_param_uint_default(dev, "wti-count", &pv->wti_count, 0);
# endif

# ifdef CONFIG_ARCH_SMP
  /* enable WTI 0 for all outputs, used to start cpus */
  for (i = 0; i < pv->irq_count; i++)
    cpu_mem_write_32(XICU_REG_ADDR(pv->addr, XICU_MSK_WTI_ENABLE, i), endian_le32(1));
# endif

#endif

#ifdef CONFIG_DRIVER_SOCLIB_XICU_TIMER
  pv->pti_count = pti_count;
  for (i = 0; i < pti_count; i++)
    {
      struct soclib_xicu_pti_s *p = pv->pti + i;
      p->start_count = 0;
      cpu_mem_read_32(XICU_REG_ADDR(pv->addr, XICU_PTI_ACK, i));
      cpu_mem_write_32(XICU_REG_ADDR(pv->addr, XICU_PTI_PER, i), 0);

# ifdef CONFIG_DRIVER_SOCLIB_XICU_ICU
      dev_request_pqueue_init(&p->queue);
      p->period = resolution;
      p->value = 0;
      p->rev = 1;
      // FIXME timer irq routing
      cpu_mem_write_32(XICU_REG_ADDR(pv->addr, XICU_MSK_PTI_ENABLE, i), 0xffffffff);
# endif
    }
#endif

  dev->drv = &soclib_xicu_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;
  return 0;

#ifdef CONFIG_DRIVER_SOCLIB_XICU_ICU
 err_unlink:
  device_irq_source_unlink(dev, pv->srcs, pv->irq_count);
 err_mem2:
  if (pv->srcs)
    mem_free(pv->srcs);
#endif
 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(soclib_xicu_cleanup)
{
  struct soclib_xicu_private_s *pv = dev->drv_pv;

#ifdef CONFIG_DRIVER_SOCLIB_XICU_ICU
  uint_fast8_t i;
# ifdef CONFIG_DRIVER_SOCLIB_XICU_TIMER
  for (i = 0; i < pv->pti_count; i++)
    {
      struct soclib_xicu_pti_s *p = pv->pti + i;
      if (!dev_request_pqueue_isempty(&p->queue))
        return -EBUSY;
    }

  for (i = 0; i < pv->pti_count; i++)
    {
      struct soclib_xicu_pti_s *p = pv->pti + i;
      cpu_mem_write_32(XICU_REG_ADDR(pv->addr, XICU_MSK_PTI_ENABLE, i), 0);
      dev_request_pqueue_destroy(&p->queue);
    }
# endif

  /* detach soclib_xicu irq end-points */
  device_irq_source_unlink(dev, pv->srcs, pv->irq_count);

  if (pv->srcs)
    mem_free(pv->srcs);
  if (pv->sinks)
    mem_free(pv->sinks);
#endif

  mem_free(pv);
  return 0;
}

