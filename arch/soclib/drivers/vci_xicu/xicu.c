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
#include <device/driver.h>
#include <device/class/icu.h>
#include <device/class/timer.h>
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

#ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_TIMER

# define SOCLIB_XICU_PTI_MIN_PERIOD 2000
# define SOCLIB_XICU_PTI_DEFAULT_PERIOD 250000

struct soclib_xicu_pti_s
{
#ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_ICU
  dev_timer_queue_root_t queue;
  dev_timer_value_t value;
  dev_timer_res_t   period;
#endif
  uint_fast8_t start_count;
};
#endif

struct soclib_xicu_private_s
{
  uintptr_t addr;

#ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_ICU
  uintptr_t hwi_count;
  struct soclib_xicu_sink_s *sinks;

  uintptr_t irq_count;
  struct dev_irq_ep_s *srcs;

# ifdef CONFIG_HEXO_IPI
  uintptr_t wti_count;
# endif
#endif

#ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_TIMER
  uintptr_t pti_count;
  struct soclib_xicu_pti_s pti[0];
#endif

};

static void soclib_xicu_pti_irq_process(struct device_s *dev, uint_fast8_t number);


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

#ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_TIMER
    if (XICU_PRIO_HAS_PTI(prio))
      {
        done = 0;
        uint_fast8_t i = XICU_PRIO_PTI(prio);
        soclib_xicu_pti_irq_process(dev, i);
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

#endif

/************************************************************************
        Timer driver part
************************************************************************/

/*

  When interruptions are available, the timer value is a 64 bits
  software counter and the hardware timer period is used as an input
  frequency divider configured from current resolution.

  When interruptions support is disabled in the configuration, the
  timer value is taken from the hardware timer register, the
  resolution is fixed to 1 and the max timer value is 2^32-1.

*/

#ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_TIMER

static DEVTIMER_REQUEST(soclib_xicu_timer_request)
{
# ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_ICU
  struct device_s *dev = tdev->dev;
  struct soclib_xicu_private_s *pv = dev->drv_pv;
  error_t err = 0;

  if (tdev->number >= pv->pti_count)
    return -ENOENT;

  if (!rq)
    return 0;

  rq->tdev = tdev;

  struct soclib_xicu_pti_s *p = pv->pti + tdev->number;

  LOCK_SPIN_IRQ(&dev->lock);

  uint64_t val = p->value;

  if (cancel)
    {
      if (rq->drvdata == p)
        {
          dev_timer_queue_remove(&p->queue, rq);
          rq->drvdata = 0;

          // stop timer if not in use
          if (--p->start_count == 0)
            cpu_mem_write_32(XICU_REG_ADDR(pv->addr, XICU_PTI_PER, tdev->number), 0);
        }
      else
        {
          err = -ETIMEDOUT;
        }
    }
  else
    {
      do {
        if (rq->delay)
          rq->deadline = val + rq->delay;

        if (rq->deadline <= val)
          {
            if (rq->callback(rq))
              continue;
            err = ETIMEDOUT;
            goto done;
          }
      } while (0);

      rq->drvdata = p;
      dev_timer_queue_insert_ascend(&p->queue, rq);

      /* start timer if needed */
      if (p->start_count++ == 0)
        {
          cpu_mem_write_32(XICU_REG_ADDR(pv->addr, XICU_PTI_VAL, tdev->number), endian_le32(p->period));
          cpu_mem_write_32(XICU_REG_ADDR(pv->addr, XICU_PTI_PER, tdev->number), endian_le32(p->period));
        }
    }
 done:

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
# else
  return -ENOTSUP;
# endif
}

# ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_ICU
static void soclib_xicu_pti_irq_process(struct device_s *dev, uint_fast8_t number)
{
  struct soclib_xicu_private_s *pv = dev->drv_pv;
  struct soclib_xicu_pti_s *p = pv->pti + number;

  lock_spin(&dev->lock);

  p->value++;

  struct dev_timer_rq_s *rq = dev_timer_queue_head(&p->queue);

  while (rq)
    {
      if (rq->deadline > p->value)
        break;

      rq->drvdata = 0;
      dev_timer_queue_pop(&p->queue);

      if (rq->callback(rq))
        {
          if (rq->delay)
            rq->deadline = p->value + rq->delay;

          rq->drvdata = p;
          dev_timer_queue_insert_ascend(&p->queue, rq);
        }
      else
        {
          // stop timer if not in use
          if (--p->start_count == 0)
            cpu_mem_write_32(XICU_REG_ADDR(pv->addr, XICU_PTI_PER, number), 0);
        }

      rq = dev_timer_queue_head(&p->queue);
    }

  cpu_mem_read_32(XICU_REG_ADDR(pv->addr, XICU_PTI_ACK, number));

  lock_release(&dev->lock);
}
# endif

static DEVTIMER_START_STOP(soclib_xicu_timer_start_stop)
{
  struct device_s *dev = tdev->dev;
  struct soclib_xicu_private_s *pv = dev->drv_pv;

  if (tdev->number >= pv->pti_count)
    return -ENOENT;

  error_t err = 0;

  struct soclib_xicu_pti_s *p = pv->pti + tdev->number;

  LOCK_SPIN_IRQ(&dev->lock);

  if (start)
    {
      if (p->start_count++ == 0)
        {
# ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_ICU
          cpu_mem_write_32(XICU_REG_ADDR(pv->addr, XICU_PTI_VAL, tdev->number), endian_le32(p->period));
          cpu_mem_write_32(XICU_REG_ADDR(pv->addr, XICU_PTI_PER, tdev->number), endian_le32(p->period));
# else
          cpu_mem_write_32(XICU_REG_ADDR(pv->addr, XICU_PTI_VAL, tdev->number), 0xffffffff);
          cpu_mem_write_32(XICU_REG_ADDR(pv->addr, XICU_PTI_PER, tdev->number), 0xffffffff);
# endif
        }
    }
  else
    {
      if ((p->start_count == 0)
# ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_ICU
          || (p->start_count == 1 && !dev_timer_queue_isempty(&p->queue))
#endif
          )
        err = -EINVAL;
      else if (--p->start_count == 0)
        cpu_mem_write_32(XICU_REG_ADDR(pv->addr, XICU_PTI_PER, tdev->number), 0);
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEVTIMER_GET_VALUE(soclib_xicu_timer_get_value)
{
  struct device_s *dev = tdev->dev;
  struct soclib_xicu_private_s *pv = dev->drv_pv;

  if (tdev->number >= pv->pti_count)
    return -ENOENT;

# ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_ICU
  struct soclib_xicu_pti_s *p = pv->pti + tdev->number;

  LOCK_SPIN_IRQ(&dev->lock);
  *value = p->value;
  LOCK_RELEASE_IRQ(&dev->lock);
#else
  *value = ~endian_le32(cpu_mem_read_32(XICU_REG_ADDR(pv->addr, XICU_PTI_VAL, tdev->number)));
#endif

  return 0;
}

static DEVTIMER_RESOLUTION(soclib_xicu_timer_resolution)
{
  struct device_s *dev = tdev->dev;
  struct soclib_xicu_private_s *pv = dev->drv_pv;

  if (tdev->number >= pv->pti_count)
    return -ENOENT;

  error_t err = 0;

# ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_ICU
  struct soclib_xicu_pti_s *p = pv->pti + tdev->number;

  LOCK_SPIN_IRQ(&dev->lock);

  if (res)
    {
      if (*res)
        {
          if (p->start_count)
            {
              err = -EBUSY;
            }
          else if (*res < SOCLIB_XICU_PTI_MIN_PERIOD)
            {
              p->period = SOCLIB_XICU_PTI_MIN_PERIOD;
              err = -ERANGE;
            }
          else
            {
              p->period = *res;
            }
        }
      *res = p->period;
    }

  if (max)
    *max = 0xffffffffffffffffULL;

  LOCK_RELEASE_IRQ(&dev->lock);

#else
  if (res)
    {
      if (*res != 0)
        err = -ENOTSUP;
      *res = 1;
    }

  if (max)
    *max = 0xffffffff;
#endif

  return err;
}

const struct driver_timer_s  soclib_xicu_timer_drv =
{
  .class_         = DRIVER_CLASS_TIMER,
  .f_request      = soclib_xicu_timer_request,
  .f_start_stop   = soclib_xicu_timer_start_stop,
  .f_get_value    = soclib_xicu_timer_get_value,
  .f_resolution   = soclib_xicu_timer_resolution,
};

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
#ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_TIMER
    &soclib_xicu_timer_drv,
#endif
    0
  }
};

static DEV_INIT(soclib_xicu_init)
{
  struct soclib_xicu_private_s  *pv;
  uint_fast8_t i;

  uintptr_t pti_count = 0;
#ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_TIMER
  device_get_param_uint_default(dev, "pti-count", &pti_count, 0);

# ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_ICU
  dev_timer_res_t resolution = SOCLIB_XICU_PTI_DEFAULT_PERIOD;
  device_get_param_uint(dev, "period", &resolution);
  if (resolution < SOCLIB_XICU_PTI_MIN_PERIOD)
    resolution = SOCLIB_XICU_PTI_MIN_PERIOD;
# endif
#endif

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  pv = mem_alloc(sizeof (*pv) + pti_count * sizeof(struct soclib_xicu_pti_s), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;
  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

#ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_ICU

  device_get_param_uint_default(dev, "irq-count", &pv->irq_count, 1);
  if (pv->irq_count)
    {
      /* init soclib_xicu irq source end-points */
      pv->srcs = mem_alloc(sizeof(pv->srcs[0]) * pv->irq_count, (mem_scope_sys));
      if (!pv->srcs)
        goto err_mem;

      device_irq_source_init(dev, pv->srcs, pv->irq_count, &soclib_xicu_source_process);

# ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_TIMER
      if (device_irq_source_link(dev, pv->srcs, pv->irq_count, 1))
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
          device_irq_sink_init(dev, &pv->sinks[i].sink, 1);
          pv->sinks[i].affinity = 0;
        }
    }

# ifdef CONFIG_HEXO_IPI
  device_get_param_uint_default(dev, "wti-count", &pv->wti_count, 0);
# endif

#endif

#ifdef CONFIG_ARCH_SMP
  /* enable WTI 0 for all outputs, used to start cpus */
  for (i = 0; i < pv->irq_count; i++)
    cpu_mem_write_32(XICU_REG_ADDR(pv->addr, XICU_MSK_WTI_ENABLE, i), endian_le32(1));
#endif

#ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_TIMER
  pv->pti_count = pti_count;
  for (i = 0; i < pti_count; i++)
    {
      struct soclib_xicu_pti_s *p = pv->pti + i;
      p->start_count = 0;
      cpu_mem_read_32(XICU_REG_ADDR(pv->addr, XICU_PTI_ACK, i));
      cpu_mem_write_32(XICU_REG_ADDR(pv->addr, XICU_PTI_PER, i), 0);

# ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_ICU
      dev_timer_queue_init(&p->queue);
      p->period = resolution;
      p->value = 0;
      // FIXME timer irq routing
      cpu_mem_write_32(XICU_REG_ADDR(pv->addr, XICU_MSK_PTI_ENABLE, i), 0xffffffff);
# endif
    }
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

