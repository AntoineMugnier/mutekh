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

#include <string.h>
#include <stdio.h>

#include <hexo/types.h>
#include <hexo/iospace.h>
#include <hexo/endian.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/class/timer.h>
#include <device/irq.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include "rttimer_regs.h"

#define RT_TIMER_REG_ADDR(a, r, n)  ((a) + (r) + (n) * 0x10)

#if defined(CONFIG_ARCH_SOCLIB)
# define RT_TIMER_ENDIAN32(x) endian_le32(x)
#elif defined(CONFIG_ARCH_GAISLER)
# define RT_TIMER_ENDIAN32(x) endian_be32(x)
#endif

struct enst_rttimer_state_s
{
#ifdef CONFIG_DEVICE_IRQ
  dev_timer_queue_root_t queue;
#endif
};

struct enst_rttimer_private_s
{
  uintptr_t addr;
  uint_fast8_t t_count;         // timers count
  uint_fast8_t start_count;
  struct dev_irq_ep_s *irq_eps;
  struct enst_rttimer_state_s t[0];
};

#ifdef CONFIG_DEVICE_IRQ
static inline void enst_rttimer_irq_process(struct enst_rttimer_private_s *pv, uint_fast8_t number)
{
  struct enst_rttimer_state_s *p = pv->t + number;
  struct dev_timer_rq_s *rq;

  while ((rq = dev_timer_queue_head(&p->queue)))
    {
      uint64_t value = RT_TIMER_ENDIAN32(cpu_mem_read_32(pv->addr + RT_TIMER_RTCL_ADDR));
      value |= (uint64_t)RT_TIMER_ENDIAN32(cpu_mem_read_32(pv->addr + RT_TIMER_RTCTMP_ADDR)) << 32;

      if (rq->deadline > value)
	{
	  cpu_mem_write_32(pv->addr + RT_TIMER_RTCTMP_ADDR, RT_TIMER_ENDIAN32(rq->deadline >> 32));
	  cpu_mem_write_32(RT_TIMER_REG_ADDR(pv->addr, RT_TIMER_DLN1_ADDR, number), RT_TIMER_ENDIAN32(rq->deadline));
	  break;
	}

      rq->drvdata = 0;
      dev_timer_queue_pop(&p->queue);

      if (rq->callback(rq, 0))
	{
	  if (rq->delay)
	    rq->deadline += rq->delay;

	  rq->drvdata = p;
	  dev_timer_queue_insert_ascend(&p->queue, rq);
	}
      else
	--pv->start_count;
    }

  // stop timer if not in use
  if (!rq && !pv->start_count)
    cpu_mem_write_32(pv->addr + RT_TIMER_CTRL_ADDR, RT_TIMER_ENDIAN32(RT_TIMER_CTRL_IEW_SMASK));
}

static DEV_IRQ_EP_PROCESS(enst_rttimer_irq_single)
{
  struct device_s *dev = ep->dev;
  struct enst_rttimer_private_s *pv = dev->drv_pv;

  lock_spin(&dev->lock);

  while (1)
    {
      uint32_t ip = cpu_mem_read_32(pv->addr + RT_TIMER_IP_ADDR);
      if (!ip)
	break;

      cpu_mem_write_32(pv->addr + RT_TIMER_IP_ADDR, ip);
      ip = RT_TIMER_ENDIAN32(ip);

      while (ip)
	{
	  uint_fast8_t number = __builtin_ctz(ip);

	  assert(number < pv->t_count);

	  enst_rttimer_irq_process(pv, number);

	  ip &= ip - 1;
	}
    }

  lock_release(&dev->lock);
}

static DEV_IRQ_EP_PROCESS(enst_rttimer_irq_separate)
{
  struct device_s *dev = ep->dev;
  struct enst_rttimer_private_s *pv = dev->drv_pv;
  uint_fast8_t number = ep - pv->irq_eps;

  assert(number < pv->t_count);
  lock_spin(&dev->lock);

  while (1)
    {
      uint32_t ip = RT_TIMER_ENDIAN32(cpu_mem_read_32(pv->addr + RT_TIMER_IP_ADDR)) & (1 << number);
      if (!ip)
	break;

      cpu_mem_write_32(pv->addr + RT_TIMER_IP_ADDR, RT_TIMER_ENDIAN32(ip));
      enst_rttimer_irq_process(pv, number);
    }

  lock_release(&dev->lock);
}
#endif

static DEVTIMER_CANCEL(enst_rttimer_cancel)
{
# ifdef CONFIG_DEVICE_IRQ
  struct device_s *dev = tdev->dev;
  struct enst_rttimer_private_s *pv = dev->drv_pv;
  error_t err = 0;

  if (tdev->number >= pv->t_count)
    return -ENOENT;

  if (!rq)
    return 0;

  struct enst_rttimer_state_s *p = pv->t + tdev->number;

  LOCK_SPIN_IRQ(&dev->lock);

  if (rq->drvdata == p)
    {
      struct dev_timer_rq_s *rq0 = dev_timer_queue_head(&p->queue);

      dev_timer_queue_remove(&p->queue, rq);
      rq->drvdata = 0;

      if (rq == rq0)  /* removed first ? */
        {
          cpu_mem_write_32(pv->addr + RT_TIMER_CANCEL_ADDR, RT_TIMER_ENDIAN32(1 << tdev->number));

          rq0 = dev_timer_queue_head(&p->queue);
          if (rq0)
            {
              /* schedule next rq */
              cpu_mem_write_32(pv->addr + RT_TIMER_RTCTMP_ADDR, RT_TIMER_ENDIAN32(rq0->deadline >> 32));
              cpu_mem_write_32(RT_TIMER_REG_ADDR(pv->addr, RT_TIMER_DLN1_ADDR, tdev->number), RT_TIMER_ENDIAN32(rq0->deadline));
            }

          /* stop timer if not in use */
          if (--pv->start_count == 0)
            cpu_mem_write_32(pv->addr + RT_TIMER_CTRL_ADDR, RT_TIMER_ENDIAN32(RT_TIMER_CTRL_IEW_SMASK));
        }
    }
  else
    {
      err = -ETIMEDOUT;
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
# else
  return -ENOTSUP;
# endif
}

static DEVTIMER_REQUEST(enst_rttimer_request)
{
# ifdef CONFIG_DEVICE_IRQ
  struct device_s *dev = tdev->dev;
  struct enst_rttimer_private_s *pv = dev->drv_pv;
  error_t err = 0;

  if (tdev->number >= pv->t_count)
    return -ENOENT;

  if (!rq)
    return 0;

  rq->tdev = tdev;

  struct enst_rttimer_state_s *p = pv->t + tdev->number;

  LOCK_SPIN_IRQ(&dev->lock);

  uint64_t value = RT_TIMER_ENDIAN32(cpu_mem_read_32(pv->addr + RT_TIMER_RTCL_ADDR));
  value |= (uint64_t)RT_TIMER_ENDIAN32(cpu_mem_read_32(pv->addr + RT_TIMER_RTCTMP_ADDR)) << 32;

  do {
    if (rq->delay)
      rq->deadline = value + rq->delay;

    if (rq->deadline <= value)
      {
        if (rq->callback(rq, 1))
          continue;
        err = ETIMEDOUT;
        goto done;
      }
  } while (0);

  rq->drvdata = p;
  dev_timer_queue_insert_ascend(&p->queue, rq);

  /* adjust earliest deadline if needed */
  if (dev_timer_queue_head(&p->queue) == rq)
    {
      cpu_mem_write_32(pv->addr + RT_TIMER_RTCTMP_ADDR, RT_TIMER_ENDIAN32(rq->deadline >> 32));
      cpu_mem_write_32(RT_TIMER_REG_ADDR(pv->addr, RT_TIMER_DLN1_ADDR, tdev->number), RT_TIMER_ENDIAN32(rq->deadline));
    }

  /* start timer if needed */
  if (pv->start_count++ == 0)
    {
      cpu_mem_write_32(pv->addr + RT_TIMER_CTRL_ADDR,
                       RT_TIMER_ENDIAN32(RT_TIMER_CTRL_CE_SMASK | RT_TIMER_CTRL_IEW_SMASK));
    }

done:;
  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
# else
  return -ENOTSUP;
# endif
}

static DEVTIMER_START_STOP(enst_rttimer_state_start_stop)
{
  struct device_s *dev = tdev->dev;
  struct enst_rttimer_private_s *pv = dev->drv_pv;

  if (tdev->number >= pv->t_count)
    return -ENOENT;

  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

# ifdef CONFIG_DEVICE_IRQ
  if (start)
    {
      if (pv->start_count++ == 0)
        {
	  cpu_mem_write_32(pv->addr + RT_TIMER_CTRL_ADDR,
			   RT_TIMER_ENDIAN32(RT_TIMER_CTRL_CE_SMASK | RT_TIMER_CTRL_IEW_SMASK));
        }
    }
  else
    {
      if (pv->start_count == 0)
	err = -EINVAL;
      else if (--pv->start_count == 0)
	cpu_mem_write_32(pv->addr + RT_TIMER_CTRL_ADDR, RT_TIMER_ENDIAN32(RT_TIMER_CTRL_IEW_SMASK));
    }
# else
  if (start)
    {
      if (pv->start_count++ == 0)
	cpu_mem_write_32(pv->addr + RT_TIMER_CTRL_ADDR, RT_TIMER_ENDIAN32(RT_TIMER_CTRL_CE_SMASK));
    }
  else
    {
      if (--pv->start_count == 0)
	cpu_mem_write_32(pv->addr + RT_TIMER_CTRL_ADDR, 0);
    }
# endif

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEVTIMER_GET_VALUE(enst_rttimer_get_value)
{
  struct device_s *dev = tdev->dev;
  struct enst_rttimer_private_s *pv = dev->drv_pv;

  if (tdev->number >= pv->t_count)
    return -ENOENT;

  LOCK_SPIN_IRQ(&dev->lock);

  *value = RT_TIMER_ENDIAN32(cpu_mem_read_32(pv->addr + RT_TIMER_RTCL_ADDR));
  *value |= (uint64_t)RT_TIMER_ENDIAN32(cpu_mem_read_32(pv->addr + RT_TIMER_RTCTMP_ADDR)) << 32;

  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}

static DEVTIMER_RESOLUTION(enst_rttimer_resolution)
{
  struct device_s *dev = tdev->dev;
  struct enst_rttimer_private_s *pv = dev->drv_pv;

  if (tdev->number >= pv->t_count)
    return -ENOENT;

  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (res)
    {
      if (*res)
        {
          if (pv->start_count)
            {
              err = -EBUSY;
            }
          else
            {
	      cpu_mem_write_32(pv->addr + RT_TIMER_SCRLD_ADDR, RT_TIMER_ENDIAN32(*res - 1));
	      cpu_mem_write_32(pv->addr + RT_TIMER_SCCNT_ADDR, RT_TIMER_ENDIAN32(*res - 1));
            }
        }
      *res = RT_TIMER_ENDIAN32(cpu_mem_read_32(pv->addr + RT_TIMER_SCRLD_ADDR)) + 1;
    }

  if (max)
    *max = 0xffffffffffffffffULL;

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

const struct driver_timer_s  enst_rttimer_timer_drv =
{
  .class_         = DRIVER_CLASS_TIMER,
  .f_request      = enst_rttimer_request,
  .f_cancel       = enst_rttimer_cancel,
  .f_start_stop   = enst_rttimer_state_start_stop,
  .f_get_value    = enst_rttimer_get_value,
  .f_resolution   = enst_rttimer_resolution,
};

/************************************************************************/

static const struct devenum_ident_s  enst_rttimer_ids[] =
{
#ifdef CONFIG_ARCH_SOCLIB
  DEVENUM_FDTNAME_ENTRY("soclib:vci_rttimer"),
#endif
#ifdef CONFIG_ARCH_GAISLER
  DEVENUM_GAISLER_ENTRY(0x09, 0x003),
#endif
  { 0 }
};

static DEV_INIT(enst_rttimer_init);
static DEV_CLEANUP(enst_rttimer_cleanup);

const struct driver_s  enst_rttimer_drv =
{
  .desc           = "Telecom ParisTech Real-time Timer",
  .id_table       = enst_rttimer_ids,
  .f_init         = enst_rttimer_init,
  .f_cleanup      = enst_rttimer_cleanup,

  .classes        = {
    &enst_rttimer_timer_drv,
    0
  }
};


static DEV_INIT(enst_rttimer_init)
{
  struct enst_rttimer_private_s  *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  uintptr_t addr;
  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL))
    return -ENOENT;

  uint32_t cfg = RT_TIMER_ENDIAN32(cpu_mem_read_32(addr + RT_TIMER_CFG_ADDR));
  uintptr_t t_count = RT_TIMER_CFG_DL_CNT_GET(cfg);
#ifdef CONFIG_DEVICE_IRQ
  uintptr_t irq_count = RT_TIMER_CFG_SI_GET(cfg) ? t_count : 1;
#endif

  if (t_count == 0)
    return -EINVAL;    

  pv = mem_alloc(sizeof (*pv) + t_count * sizeof(struct enst_rttimer_state_s)
#ifdef CONFIG_DEVICE_IRQ
		              + irq_count * sizeof(struct dev_irq_ep_s)
#endif
                                           , (mem_scope_sys));
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));
  pv->addr = addr;
  pv->t_count = t_count;
  pv->start_count = 0;
  dev->drv_pv = pv;

  /* clear irqs */
  cpu_mem_write_32(pv->addr + RT_TIMER_IE_ADDR, 0);
  cpu_mem_write_32(pv->addr + RT_TIMER_PE_ADDR, 0);
  cpu_mem_write_32(pv->addr + RT_TIMER_IP_ADDR, 0xffffffff);

  /* set default resolution */
  dev_timer_res_t resolution = CONFIG_DRIVER_ENST_RTTIMER_PERIOD;
  device_get_param_uint(dev, "period", &resolution);
  if (resolution < 1)
    resolution = 1;
  if (resolution > 65536)
    resolution = 65536;

  cpu_mem_write_32(pv->addr + RT_TIMER_SCRLD_ADDR, RT_TIMER_ENDIAN32(resolution - 1));
  cpu_mem_write_32(pv->addr + RT_TIMER_SCCNT_ADDR, RT_TIMER_ENDIAN32(resolution - 1));

#ifdef CONFIG_DEVICE_IRQ
  pv->irq_eps = (void*)(pv->t + t_count);

  if (RT_TIMER_CFG_SI_GET(cfg))
    device_irq_source_init(dev, pv->irq_eps, irq_count, enst_rttimer_irq_separate);
  else
    device_irq_source_init(dev, pv->irq_eps, 1, enst_rttimer_irq_single);

  if (device_irq_source_link(dev, pv->irq_eps, irq_count, 1))
    goto err_mem;

  uint_fast8_t i;
  for (i = 0; i < t_count; i++)
    {
      struct enst_rttimer_state_s *p = pv->t + i;
      dev_timer_queue_init(&p->queue);
    }

  cpu_mem_write_32(pv->addr + RT_TIMER_CTRL_ADDR,
		   RT_TIMER_ENDIAN32(RT_TIMER_CTRL_IEW_SMASK | RT_TIMER_CTRL_IEA_SMASK));
#else
  cpu_mem_write_32(pv->addr + RT_TIMER_CTRL_ADDR, 0);
#endif

  dev->drv = &enst_rttimer_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;
  return 0;

#ifdef CONFIG_DEVICE_IRQ
 err_mem:
  mem_free(pv);
  return -1;
#endif
}

static DEV_CLEANUP(enst_rttimer_cleanup)
{
  struct enst_rttimer_private_s *pv = dev->drv_pv;

  /* stop */
  cpu_mem_write_32(pv->addr + RT_TIMER_CTRL_ADDR, 0);

#ifdef CONFIG_DEVICE_IRQ
  cpu_mem_write_32(pv->addr + RT_TIMER_IE_ADDR, 0);

  uint_fast8_t i;
  for (i = 0; i < pv->t_count; i++)
    {
      struct enst_rttimer_state_s *p = pv->t + i;
      dev_timer_queue_destroy(&p->queue);
    }

  device_irq_source_unlink(dev, pv->irq_eps, pv->t_count);
#endif

  mem_free(pv);
}

REGISTER_DRIVER(enst_rttimer_drv);

