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

    Copyright (c) 2013 Sebastien Cerdan <sebcerdan@gmail.com>

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
#include <device/clock.h>

#include <mutek/mem_alloc.h>
#include <mutek/kroutine.h>

#include <arch/efm32/timer.h>

#define EFM32_TIMER_HW_WIDTH 16
#define EFM32_TIMER_HW_MASK  0xffff
#define EFM32_TIMER_SW_MASK  0xffffffffffff0000ULL
#define EFM32_TIMER_TOP      0xffff
#define EFM32_TIMER_CHANNEL  0

 
struct efm32_timer_private_s
{
  /* Timer address */
  uintptr_t addr;
  /* Start timer counter, bit 0 indicates if there are pending requests */
#ifdef CONFIG_DEVICE_IRQ
  /* Timer Software value */
  uint64_t swvalue;
  /* Interrupt end-point */
  struct dev_irq_src_s irq_eps;
  /* Request queue */
  dev_request_pqueue_root_t queue;
#endif

#ifdef CONFIG_DEVICE_CLOCK
  struct dev_clock_sink_ep_s clk_ep;
  struct dev_freq_s freq;
#endif

  enum dev_timer_capabilities_e cap:8;
  uint8_t prescaler:4;
  dev_timer_cfgrev_t rev;
};

#define EFM32_TIMER_CTRL_CONST (EFM32_TIMER_CTRL_MODE(UP) |             \
                                EFM32_TIMER_CTRL_CLKSEL(PRESCHFPERCLK) | \
                                EFM32_TIMER_CTRL_SYNC(NONE) |           \
                                EFM32_TIMER_CTRL_RISEA(NONE) |          \
                                EFM32_TIMER_CTRL_FALLA(NONE))

/* This function starts the hardware timer counter. */
static inline void efm32_timer_start_counter(struct efm32_timer_private_s *pv)
{
#ifdef CONFIG_DEVICE_CLOCK_GATING
  dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER_CLOCK);
#endif
  uint32_t ctrl = EFM32_TIMER_CTRL_CONST;
  EFM32_TIMER_CTRL_PRESC_SETVAL(ctrl, pv->prescaler);
  cpu_mem_write_32(pv->addr + EFM32_TIMER_CTRL_ADDR, endian_le32(ctrl));

  cpu_mem_write_32(pv->addr + EFM32_TIMER_CMD_ADDR, endian_le32(EFM32_TIMER_CMD_START));
}

/* This function stops the hardware timer counter. */
static inline void efm32_timer_stop_counter(struct efm32_timer_private_s *pv)
{
  cpu_mem_write_32(pv->addr + EFM32_TIMER_CMD_ADDR, endian_le32(EFM32_TIMER_CMD_STOP));
#ifdef CONFIG_DEVICE_CLOCK_GATING
  if (endian_le32(cpu_mem_read_32(pv->addr + EFM32_TIMER_IF_ADDR))
      & (EFM32_TIMER_IF_OF | EFM32_TIMER_IF_CC(EFM32_TIMER_CHANNEL)))
    return;
  dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER);
#endif
}

/* This function returns a concatenation of the software timer value and 
   of the hardware timer value. If a top value overflow interrupt is pending
   in the timer, the software timer value is incremented by one to get the 
   most recent timer value. */
static uint64_t get_timer_value(struct efm32_timer_private_s *pv)
{
  uint64_t value = endian_le32(cpu_mem_read_32(pv->addr + EFM32_TIMER_CNT_ADDR));

#ifdef CONFIG_DEVICE_IRQ
  if (value < EFM32_TIMER_HW_MASK / 2)      /* check if a wrap just occured */
    {
      uint32_t x = endian_le32(cpu_mem_read_32(pv->addr + EFM32_TIMER_IF_ADDR));
      if (x & EFM32_TIMER_IF_OF)
        value += 1ULL << EFM32_TIMER_HW_WIDTH;
    }

  return value + (pv->swvalue << EFM32_TIMER_HW_WIDTH);
#else
  return value;
#endif
}

#ifdef CONFIG_DEVICE_IRQ

/* This function writes a value in the Compare/Capture channel 0 of the 
   timer. When the timer counter value will be greater than this value 
   a compare interrup will be raised. */
static inline void efm32_timer_enable_compare(struct efm32_timer_private_s *pv, dev_timer_value_t v)
{
  /* Write v in Compare 0 channel */
  cpu_mem_write_32(pv->addr + EFM32_TIMER_CC_CCV_ADDR(EFM32_TIMER_CHANNEL),
                   endian_le32(v & EFM32_TIMER_CC_CCV_MASK));

  cpu_mem_write_32(pv->addr + EFM32_TIMER_CC_CTRL_ADDR(EFM32_TIMER_CHANNEL),
                   endian_le32(EFM32_TIMER_CC_CTRL_MODE(OUTPUTCOMPARE)));
}

/* This function disables the interrupt associated to compare/capture
   channel 0. */
static inline void efm32_timer_disable_compare(struct efm32_timer_private_s *pv)
{
  cpu_mem_write_32(pv->addr + EFM32_TIMER_CC_CTRL_ADDR(EFM32_TIMER_CHANNEL), 0);
}

static bool_t efm32_timer_request_start(struct efm32_timer_private_s *pv,
                                      struct dev_timer_rq_s *rq,
                                      dev_timer_value_t value)
{
  /* enable hw comparator if software part of the counter match */
  if (((rq->deadline ^ value) & EFM32_TIMER_SW_MASK))
    return 0;

  efm32_timer_enable_compare(pv, rq->deadline);

  /* hw compare for == only, check for race condition */
  if (rq->deadline <= get_timer_value(pv))
    return 1;

  return 0;
}

static inline void efm32_timer_raise_irq(struct efm32_timer_private_s *pv)
{
  cpu_mem_write_32(pv->addr + EFM32_TIMER_IFS_ADDR,
                   endian_le32(EFM32_TIMER_IF_CC(EFM32_TIMER_CHANNEL)));
}

static DEV_IRQ_SRC_PROCESS(efm32_timer_irq)
{
  struct device_s *dev = ep->base.dev;
  struct efm32_timer_private_s *pv = dev->drv_pv;

  lock_spin(&dev->lock);
#ifdef CONFIG_DEVICE_CLOCK_GATING
  assert(dev->start_count);
#endif

  while (1)
    {
      uint32_t irq = endian_le32(cpu_mem_read_32(pv->addr + EFM32_TIMER_IF_ADDR))
        & (EFM32_TIMER_IF_OF | EFM32_TIMER_IF_CC(EFM32_TIMER_CHANNEL));

      if (!irq)
        break;

      cpu_mem_write_32(pv->addr + EFM32_TIMER_IFC_ADDR, endian_le32(irq));

      /* Compare channel interrupt */ 
      if (irq & EFM32_TIMER_IF_CC(EFM32_TIMER_CHANNEL))
        efm32_timer_disable_compare(pv);

      /* Update the software part of the counter */
      if (irq & EFM32_TIMER_IF_OF)
        pv->swvalue++;

      while (1)
        {
          struct dev_timer_rq_s *rq;
          rq = dev_timer_rq_s_cast(dev_request_pqueue_head(&pv->queue));
          if (rq == NULL)
            {
              dev->start_count &= ~1;
              if (dev->start_count == 0)
                efm32_timer_stop_counter(pv);
              break;
            }

          uint64_t value = get_timer_value(pv);

          /* setup compare for first request */
          if (rq->deadline > value)
            if (!efm32_timer_request_start(pv, rq, value))
              break;

          dev_timer_pqueue_remove(&pv->queue, dev_timer_rq_s_base(rq));
          efm32_timer_disable_compare(pv);
          rq->rq.drvdata = NULL;

          lock_release(&dev->lock);
          kroutine_exec(&rq->rq.kr);
          lock_spin(&dev->lock);
        }
    }

  lock_release(&dev->lock);
}
#endif

static DEV_TIMER_CANCEL(efm32_timer_cancel)
{
#ifdef CONFIG_DEVICE_IRQ
  struct device_s *dev = accessor->dev;
  struct efm32_timer_private_s *pv = dev->drv_pv;
  error_t err = -ETIMEDOUT;

  LOCK_SPIN_IRQ(&dev->lock);

  if (rq->rq.drvdata == pv)
    {
      struct dev_timer_rq_s *rqnext = NULL;
      bool_t first = (dev_request_pqueue_prev(&pv->queue, dev_timer_rq_s_base(rq)) == NULL);

      if (first)
        rqnext = dev_timer_rq_s_cast(dev_request_pqueue_next(&pv->queue, dev_timer_rq_s_base(rq)));

      dev_timer_pqueue_remove(&pv->queue, dev_timer_rq_s_base(rq));
      rq->rq.drvdata = NULL;

      if (first)
        {
          efm32_timer_disable_compare(pv);

          if (rqnext != NULL)
            {
              /* start next request, raise irq on race condition */
              if (efm32_timer_request_start(pv, rqnext, get_timer_value(pv)))
                efm32_timer_raise_irq(pv);
            }
          else
            {
              dev->start_count &= ~1;
              if (dev->start_count == 0)
                efm32_timer_stop_counter(pv);
            }
        }

      err = 0;
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
#else
  return -ENOTSUP;
#endif
}

static DEV_TIMER_REQUEST(efm32_timer_request)
{
#ifdef CONFIG_DEVICE_IRQ
  struct device_s *dev = accessor->dev;
  struct efm32_timer_private_s *pv = dev->drv_pv;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (rq->rev && rq->rev != pv->rev)
    err = -EAGAIN;
  else
    {
      /* Start timer if needed */
      if (dev->start_count == 0)
        efm32_timer_start_counter(pv);

      uint64_t value = get_timer_value(pv);

      if (rq->delay)
        rq->deadline = value + rq->delay;

      if (rq->deadline <= value)
        err = -ETIMEDOUT;
      else
        {
          dev->start_count |= 1;
          dev_timer_pqueue_insert(&pv->queue, dev_timer_rq_s_base(rq));
          rq->rq.drvdata = pv;

          /* start request, raise irq on race condition */
          if (dev_request_pqueue_prev(&pv->queue, dev_timer_rq_s_base(rq)) == NULL)
            if (efm32_timer_request_start(pv, rq, value))
              efm32_timer_raise_irq(pv);
        }

      if (dev->start_count == 0)
        efm32_timer_stop_counter(pv);
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
#else
  return -ENOTSUP;
#endif
}

static DEV_USE(efm32_timer_use)
{
  struct device_accessor_s *accessor = param;

  switch (op)
    {
#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
    case DEV_USE_CLOCK_NOTIFY: {
      struct dev_clock_notify_s *chg = param;
      struct dev_clock_sink_ep_s *sink = chg->sink;
      struct device_s *dev = sink->dev;
      struct efm32_timer_private_s *pv = dev->drv_pv;
      pv->freq = chg->freq;
      pv->rev += 2;
      return 0;
    }
#endif

    case DEV_USE_START: {
      struct device_s *dev = accessor->dev;
      struct efm32_timer_private_s *pv = dev->drv_pv;
      if (dev->start_count == 0)
        efm32_timer_start_counter(pv);
      return 0;
    }

    case DEV_USE_STOP: {
      struct device_s *dev = accessor->dev;
      struct efm32_timer_private_s *pv = dev->drv_pv;
      if (dev->start_count == 0)
        efm32_timer_stop_counter(pv);
      return 0;
    }

    default:
      return dev_use_generic(param, op);
    }
}

static DEV_TIMER_GET_VALUE(efm32_timer_get_value)
{
  struct device_s *dev = accessor->dev;
  struct efm32_timer_private_s *pv = dev->drv_pv;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

#ifdef CONFIG_DEVICE_CLOCK_GATING
  if (!dev->start_count)
    err = -EBUSY;
  else
#endif
    *value = get_timer_value(pv);

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEV_TIMER_CONFIG(efm32_timer_config)
{
  struct device_s *dev = accessor->dev;
  struct efm32_timer_private_s *pv = dev->drv_pv;
  uint32_t r, div;

  error_t err = 0;

#ifndef CONFIG_DEVICE_CLOCK
  if (cfg)
    {
      if (device_get_res_freq(accessor->dev, &cfg->freq, 0))
        cfg->freq = DEV_FREQ_INVALID;
    }
#endif

  LOCK_SPIN_IRQ(&dev->lock);

#ifdef CONFIG_DEVICE_CLOCK
  if (cfg)
    cfg->freq = pv->freq;
#endif

  if (res)
    {
      if (dev->start_count)
        {
          err = -EBUSY;
          r = res;
        }
      else
        {
          /* div is either set to maximum value 10 or rounded down to the nearest power of 2 */
          div = res > 1024 ? 10 : sizeof(__compiler_sint_t) * 8 - __builtin_clz(res) - 1;
          pv->prescaler = div;

          r = 1 << div;
          if (r != res)
            err = -ERANGE;

          pv->rev += 2;
        }
    }
  else
    {
      r = 1 << pv->prescaler;
    }

  if (cfg)
    {
      cfg->rev = pv->rev;
      cfg->res = r;
      cfg->cap = pv->cap;
#ifdef CONFIG_DEVICE_IRQ
      cfg->max = 0xffffffffffffffffULL;
#else
      cfg->max = 0xffff;
#endif
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

/************************************************************************/

static DEV_INIT(efm32_timer_init);
static DEV_CLEANUP(efm32_timer_cleanup);

DRIVER_DECLARE(efm32_timer_drv, 0, "EFM32 Timer", efm32_timer,
               DRIVER_TIMER_METHODS(efm32_timer));

DRIVER_REGISTER(efm32_timer_drv);

static DEV_INIT(efm32_timer_init)
{
  struct efm32_timer_private_s  *pv;
  uintptr_t addr;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL))
    return -ENOENT;

  pv = mem_alloc(sizeof(struct efm32_timer_private_s), (mem_scope_sys));

  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));
  pv->addr = addr;
  pv->rev = 1;
  pv->cap = DEV_TIMER_CAP_STOPPABLE | DEV_TIMER_CAP_HIGHRES | DEV_TIMER_CAP_KEEPVALUE | DEV_TIMER_CAP_TICKLESS;
  dev->drv_pv = pv;

#ifdef CONFIG_DEVICE_CLOCK
  /* enable clock */
  dev_clock_sink_init(dev, &pv->clk_ep, DEV_CLOCK_EP_SINK_NOTIFY |
                      DEV_CLOCK_EP_POWER_CLOCK | DEV_CLOCK_EP_SINK_SYNC);

  if (dev_clock_sink_link(&pv->clk_ep, 0, &pv->freq))
    goto err_mem;

# ifdef CONFIG_DEVICE_CLOCK_VARFREQ
  if (pv->clk_ep.flags & DEV_CLOCK_EP_VARFREQ)
    pv->cap |= DEV_TIMER_CAP_VARFREQ | DEV_TIMER_CAP_CLKSKEW;
# endif
#endif

#ifdef CONFIG_DEVICE_IRQ
  pv->cap |= DEV_TIMER_CAP_REQUEST;

  device_irq_source_init(dev, &pv->irq_eps, 1, efm32_timer_irq);

  if (device_irq_source_link(dev, &pv->irq_eps, 1, 1))
    goto err_clk;

  dev_request_pqueue_init(&pv->queue);
#endif

  /* Stop timer */
  cpu_mem_write_32(pv->addr + EFM32_TIMER_CMD_ADDR, endian_le32(EFM32_TIMER_CMD_STOP));

#ifdef CONFIG_DEVICE_IRQ
  /* Clear interrupts */
  cpu_mem_write_32(pv->addr + EFM32_TIMER_IFC_ADDR, endian_le32(EFM32_TIMER_IFC_MASK));
  
  /* Enable Overflow interrupts */
  cpu_mem_write_32(pv->addr + EFM32_TIMER_IEN_ADDR, endian_le32(EFM32_TIMER_IEN_OF |
                                               EFM32_TIMER_IEN_CC(EFM32_TIMER_CHANNEL)));

  /* Set Compare/Capture channel 0 to Output Compare */
  cpu_mem_write_32(pv->addr + EFM32_TIMER_CC_CTRL_ADDR(EFM32_TIMER_CHANNEL), 0);

  pv->swvalue = 0;
#else
  cpu_mem_write_32(pv->addr + EFM32_TIMER_IEN_ADDR, 0);
#endif

  pv->prescaler = EFM32_TIMER_CTRL_PRESC_DIV1024;

  /* Ctrl register configuration */
  cpu_mem_write_32(pv->addr + EFM32_TIMER_CTRL_ADDR, endian_le32(EFM32_TIMER_CTRL_CONST));

  /* Set counter wrapping value to EFM32_TIMER_TOP */
  cpu_mem_write_32(pv->addr + EFM32_TIMER_TOP_ADDR, endian_le32(EFM32_TIMER_TOP));

#ifdef CONFIG_DEVICE_CLOCK_GATING
  dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER);
#endif

  return 0;

 err_clk:
#ifdef CONFIG_DEVICE_IRQ
# ifdef CONFIG_DEVICE_CLOCK
  dev_clock_sink_unlink(&pv->clk_ep);
# endif
#endif
 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(efm32_timer_cleanup)
{
  struct efm32_timer_private_s *pv = dev->drv_pv;

#ifdef CONFIG_DEVICE_CLOCK
  dev_clock_sink_unlink(&pv->clk_ep);
#endif

#ifdef CONFIG_DEVICE_IRQ
  dev_request_pqueue_destroy(&pv->queue);

  device_irq_source_unlink(dev, &pv->irq_eps, 1);
#endif

  mem_free(pv);

  return 0;
}

