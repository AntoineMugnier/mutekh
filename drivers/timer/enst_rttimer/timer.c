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
#else
# error
#endif

struct enst_rttimer_state_s
{
#ifdef CONFIG_DEVICE_IRQ
  dev_request_pqueue_root_t queue;
#endif
};

struct enst_rttimer_private_s
{
  uintptr_t addr;
  uint_fast8_t t_count;         // timers count
  dev_timer_cfgrev_t rev;
#ifdef CONFIG_DEVICE_IRQ
  struct dev_irq_src_s *irq_eps;
#endif
  struct enst_rttimer_state_s t[0];
};

#ifdef CONFIG_DEVICE_IRQ
static inline void enst_rttimer_irq_process(struct device_s *dev, uint_fast8_t number)
{
  struct enst_rttimer_private_s *pv = dev->drv_pv;
  struct enst_rttimer_state_s *p = pv->t + number;

  struct dev_timer_rq_s *rq;

  while ((rq = dev_timer_rq_s_cast(dev_request_pqueue_head(&p->queue))))
    {
      assert(dev->start_count >= 1);

      uint64_t value = RT_TIMER_ENDIAN32(cpu_mem_read_32(pv->addr + RT_TIMER_RTCL_ADDR));
      value |= (uint64_t)RT_TIMER_ENDIAN32(cpu_mem_read_32(pv->addr + RT_TIMER_RTCTMP_ADDR)) << 32;

      if (rq->deadline > value)
	{
	  cpu_mem_write_32(pv->addr + RT_TIMER_RTCTMP_ADDR, RT_TIMER_ENDIAN32(rq->deadline >> 32));
	  cpu_mem_write_32(RT_TIMER_REG_ADDR(pv->addr, RT_TIMER_DLN1_ADDR, number), RT_TIMER_ENDIAN32(rq->deadline));
	  break;
	}

      rq->rq.drvdata = NULL;
      dev_timer_pqueue_remove(&p->queue, dev_timer_rq_s_base(rq));

      lock_release(&dev->lock);
      kroutine_exec(&rq->rq.kr);
      lock_spin(&dev->lock);

      dev->start_count--;
      if (dev->start_count == 0)
        cpu_mem_write_32(pv->addr + RT_TIMER_CTRL_ADDR, RT_TIMER_ENDIAN32(RT_TIMER_CTRL_IEW_SMASK));
    }
}

static DEV_IRQ_SRC_PROCESS(enst_rttimer_irq_single)
{
  struct device_s *dev = ep->base.dev;
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

	  enst_rttimer_irq_process(dev, number);

	  ip &= ip - 1;
	}
    }

  lock_release(&dev->lock);
}

static DEV_IRQ_SRC_PROCESS(enst_rttimer_irq_separate)
{
  struct device_s *dev = ep->base.dev;
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
      enst_rttimer_irq_process(dev, number);
    }

  lock_release(&dev->lock);
}
#endif

static DEV_TIMER_CANCEL(enst_rttimer_cancel)
{
# ifdef CONFIG_DEVICE_IRQ
  struct device_s *dev = accessor->dev;
  struct enst_rttimer_private_s *pv = dev->drv_pv;
  error_t err = 0;

  if (accessor->number >= pv->t_count)
    return -ENOTSUP;

  struct enst_rttimer_state_s *p = pv->t + accessor->number;

  LOCK_SPIN_IRQ(&dev->lock);

  if (rq->rq.drvdata == p)
    {
      struct dev_timer_rq_s *rqnext = NULL;
      bool_t first = (dev_request_pqueue_prev(&p->queue, dev_timer_rq_s_base(rq)) == NULL);

      if (first)
        rqnext = dev_timer_rq_s_cast(dev_request_pqueue_next(&p->queue, dev_timer_rq_s_base(rq)));

      dev_timer_pqueue_remove(&p->queue, dev_timer_rq_s_base(rq));
      rq->rq.drvdata = NULL;

      /* stop timer if not in use */
      dev->start_count--;
      if (dev->start_count == 0)
        cpu_mem_write_32(pv->addr + RT_TIMER_CTRL_ADDR, RT_TIMER_ENDIAN32(RT_TIMER_CTRL_IEW_SMASK));

      if (first)  /* removed first ? */
        {
          cpu_mem_write_32(pv->addr + RT_TIMER_CANCEL_ADDR, RT_TIMER_ENDIAN32(1 << accessor->number));

          if (rqnext != NULL)
            {
              /* schedule next rq */
              cpu_mem_write_32(pv->addr + RT_TIMER_RTCTMP_ADDR, RT_TIMER_ENDIAN32(rqnext->deadline >> 32));
              cpu_mem_write_32(RT_TIMER_REG_ADDR(pv->addr, RT_TIMER_DLN1_ADDR, accessor->number), RT_TIMER_ENDIAN32(rqnext->deadline));
            }
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

static DEV_TIMER_REQUEST(enst_rttimer_request)
{
# ifdef CONFIG_DEVICE_IRQ
  struct device_s *dev = accessor->dev;
  struct enst_rttimer_private_s *pv = dev->drv_pv;
  error_t err = 0;

  if (accessor->number >= pv->t_count)
    return -ENOTSUP;

  struct enst_rttimer_state_s *p = pv->t + accessor->number;

  LOCK_SPIN_IRQ(&dev->lock);

  if (rq->rev && rq->rev != pv->rev)
    err = -EAGAIN;
  else
    {
      uint64_t value = RT_TIMER_ENDIAN32(cpu_mem_read_32(pv->addr + RT_TIMER_RTCL_ADDR));
      value |= (uint64_t)RT_TIMER_ENDIAN32(cpu_mem_read_32(pv->addr + RT_TIMER_RTCTMP_ADDR)) << 32;

      if (rq->delay)
        rq->deadline = value + rq->delay;

      if (rq->deadline <= value)
        err = -ETIMEDOUT;
      else
        {
          rq->rq.drvdata = p;
          dev_timer_pqueue_insert(&p->queue, dev_timer_rq_s_base(rq));

          /* adjust earliest deadline if needed */
          if (dev_request_pqueue_prev(&p->queue, dev_timer_rq_s_base(rq)) == NULL)
            {
              cpu_mem_write_32(pv->addr + RT_TIMER_RTCTMP_ADDR, RT_TIMER_ENDIAN32(rq->deadline >> 32));
              cpu_mem_write_32(RT_TIMER_REG_ADDR(pv->addr, RT_TIMER_DLN1_ADDR, accessor->number), RT_TIMER_ENDIAN32(rq->deadline));
            }

          /* start timer if needed */
          if (dev->start_count == 0)
            {
              cpu_mem_write_32(pv->addr + RT_TIMER_CTRL_ADDR,
                               RT_TIMER_ENDIAN32(RT_TIMER_CTRL_CE_SMASK | RT_TIMER_CTRL_IEW_SMASK));
            }
          dev->start_count++;
        }
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
# else
  return -ENOTSUP;
# endif
}

static DEV_USE(enst_rttimer_use)
{
  struct device_accessor_s *accessor = param;

  switch (op)
    {
    case DEV_USE_GET_ACCESSOR: {
      struct enst_rttimer_private_s *pv = accessor->dev->drv_pv;
      if (accessor->number >= pv->t_count)
        return -ENOTSUP;
    }

    case DEV_USE_LAST_NUMBER: {
      struct enst_rttimer_private_s *pv = accessor->dev->drv_pv;
      accessor->number = pv->t_count - 1;
      return 0;
    }

    case DEV_USE_START: {
      struct device_s *dev = accessor->dev;
      struct enst_rttimer_private_s *pv = dev->drv_pv;

      if (dev->start_count == 0)
#ifdef CONFIG_DEVICE_IRQ
        cpu_mem_write_32(pv->addr + RT_TIMER_CTRL_ADDR,
                         RT_TIMER_ENDIAN32(RT_TIMER_CTRL_CE_SMASK | RT_TIMER_CTRL_IEW_SMASK));
#else
 	cpu_mem_write_32(pv->addr + RT_TIMER_CTRL_ADDR, RT_TIMER_ENDIAN32(RT_TIMER_CTRL_CE_SMASK));
#endif
      return 0;
    }

    case DEV_USE_STOP: {
      struct device_s *dev = accessor->dev;
      struct enst_rttimer_private_s *pv = dev->drv_pv;
      error_t err = 0;

      if (dev->start_count == 0)
#ifdef CONFIG_DEVICE_IRQ
        cpu_mem_write_32(pv->addr + RT_TIMER_CTRL_ADDR, RT_TIMER_ENDIAN32(RT_TIMER_CTRL_IEW_SMASK));
#else
	cpu_mem_write_32(pv->addr + RT_TIMER_CTRL_ADDR, 0);
#endif
      return err;
    }

    default:
      return dev_use_generic(param, op);
    }
}

static DEV_TIMER_GET_VALUE(enst_rttimer_get_value)
{
  struct device_s *dev = accessor->dev;
  struct enst_rttimer_private_s *pv = dev->drv_pv;
  error_t err = 0;

  if (accessor->number >= pv->t_count)
    return -ENOTSUP;

  LOCK_SPIN_IRQ(&dev->lock);

  if (rev && rev != pv->rev)
    err = -EAGAIN;
  else
    {
      *value = RT_TIMER_ENDIAN32(cpu_mem_read_32(pv->addr + RT_TIMER_RTCL_ADDR));
      *value |= (uint64_t)RT_TIMER_ENDIAN32(cpu_mem_read_32(pv->addr + RT_TIMER_RTCTMP_ADDR)) << 32;
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEV_TIMER_CONFIG(enst_rttimer_config)
{
  struct device_s *dev = accessor->dev;
  struct enst_rttimer_private_s *pv = dev->drv_pv;

  if (accessor->number >= pv->t_count)
    return -ENOTSUP;

  error_t err = 0;

  if (cfg && device_get_res_freq(accessor->dev, &cfg->freq, 0))
    cfg->freq = DEV_FREQ_INVALID;

  LOCK_SPIN_IRQ(&dev->lock);

  if (res)
    {
      if (dev->start_count)
        {
          err = -EBUSY;
        }
      else
        {
          cpu_mem_write_32(pv->addr + RT_TIMER_SCRLD_ADDR, RT_TIMER_ENDIAN32(res - 1));
          cpu_mem_write_32(pv->addr + RT_TIMER_SCCNT_ADDR, RT_TIMER_ENDIAN32(res - 1));
        }
      pv->rev += 2;
    }

  uint32_t r = RT_TIMER_ENDIAN32(cpu_mem_read_32(pv->addr + RT_TIMER_SCRLD_ADDR)) + 1;
  if (res && r != res)
    err = -ERANGE;

  if (cfg)
    {
      cfg->acc = DEV_FREQ_ACC_INVALID;
      cfg->max = 0xffffffffffffffffULL;
      cfg->rev = pv->rev;
      cfg->res = r;
      cfg->cap = DEV_TIMER_CAP_STOPPABLE | DEV_TIMER_CAP_KEEPVALUE
        | DEV_TIMER_CAP_HIGHRES | DEV_TIMER_CAP_TICKLESS;
#ifdef CONFIG_DEVICE_IRQ
      cfg->cap |= DEV_TIMER_CAP_REQUEST;
#endif
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

/************************************************************************/

static DEV_INIT(enst_rttimer_init);
static DEV_CLEANUP(enst_rttimer_cleanup);

DRIVER_DECLARE(enst_rttimer_drv, 0, "Telecom ParisTech Real-time Timer", enst_rttimer,
               DRIVER_TIMER_METHODS(enst_rttimer));

DRIVER_REGISTER(enst_rttimer_drv
#ifdef CONFIG_ARCH_SOCLIB
                , DEV_ENUM_FDTNAME_ENTRY("soclib:rttimer")
#endif
#ifdef CONFIG_ARCH_GAISLER
                , DEV_ENUM_GAISLER_ENTRY(0x09, 0x003)
#endif
                );


static DEV_INIT(enst_rttimer_init)
{
  struct enst_rttimer_private_s  *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  uintptr_t addr;
  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL))
    return -ENOTSUP;

  uint32_t cfg = RT_TIMER_ENDIAN32(cpu_mem_read_32(addr + RT_TIMER_CFG_ADDR));
  uintptr_t t_count = RT_TIMER_CFG_DL_CNT_GET(cfg);
#ifdef CONFIG_DEVICE_IRQ
  uintptr_t irq_count = RT_TIMER_CFG_SI_GET(cfg) ? t_count : 1;
#endif

  if (t_count == 0)
    return -EINVAL;

  size_t s = sizeof (*pv) + t_count * sizeof(struct enst_rttimer_state_s)
#ifdef CONFIG_DEVICE_IRQ
    + irq_count * sizeof(struct dev_irq_src_s)
#endif
    ;

  pv = mem_alloc(s, (mem_scope_sys));
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, s);
  pv->addr = addr;
  pv->t_count = t_count;
  pv->rev = 1;
  dev->drv_pv = pv;

  /* clear irqs */
  cpu_mem_write_32(pv->addr + RT_TIMER_IE_ADDR, 0);
  cpu_mem_write_32(pv->addr + RT_TIMER_PE_ADDR, 0);
  cpu_mem_write_32(pv->addr + RT_TIMER_IP_ADDR, 0xffffffff);

  /* set default resolution */
  dev_timer_res_t resolution = CONFIG_DRIVER_ENST_RTTIMER_PERIOD;
  cpu_mem_write_32(pv->addr + RT_TIMER_SCRLD_ADDR, RT_TIMER_ENDIAN32(resolution - 1));
  cpu_mem_write_32(pv->addr + RT_TIMER_SCCNT_ADDR, RT_TIMER_ENDIAN32(resolution - 1));

#ifdef CONFIG_DEVICE_IRQ
  pv->irq_eps = (void*)(pv->t + t_count);

  if (RT_TIMER_CFG_SI_GET(cfg))
    device_irq_source_init(dev, pv->irq_eps, irq_count,
                           enst_rttimer_irq_separate);
  else
    device_irq_source_init(dev, pv->irq_eps, 1,
                           enst_rttimer_irq_single);

  if (device_irq_source_link(dev, pv->irq_eps, irq_count, -1))
    goto err_mem;

  uint_fast8_t i;
  for (i = 0; i < t_count; i++)
    {
      struct enst_rttimer_state_s *p = pv->t + i;
      dev_request_pqueue_init(&p->queue);
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
      dev_request_pqueue_destroy(&p->queue);
    }

  device_irq_source_unlink(dev, pv->irq_eps, pv->t_count);
#endif

  mem_free(pv);

  return 0;
}

