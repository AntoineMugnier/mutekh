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

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/class/timer.h>
#include <device/irq.h>
#include <device/clock.h>

#include <mutek/mem_alloc.h>
#include <mutek/kroutine.h>
#include <mutek/printk.h>

#include <arch/efm32/rtc.h>
#include <arch/efm32/devaddr.h>

#if EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 0
#define EFM32_RTC_HW_WIDTH 24
#define EFM32_RTC_HW_MASK  0xffffff
#define EFM32_RTC_SW_MASK  0xffffffffff000000ULL

#elif EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 1
#define EFM32_RTC_HW_WIDTH 32
#define EFM32_RTC_HW_MASK  0xffffffff
#define EFM32_RTC_SW_MASK  0xffffffff00000000ULL

#else
# error
#endif

DRIVER_PV(struct efm32_rtc_private_s
{
#if defined(CONFIG_DEVICE_CLOCK) || EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 1
  dev_timer_cfgrev_t rev;
#endif

  /* Timer Software value */
  uint32_t swvalue;
  /* Interrupt endpoint */
  struct dev_irq_src_s irq_eps;
  /* Request queue */
  dev_request_pqueue_root_t queue;

  struct dev_clock_sink_ep_s clk_ep;
  struct dev_freq_s freq;

  enum dev_timer_capabilities_e cap:8;
#if EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 1
  uint8_t prescaler:4;
#endif
});

/* This function starts the hardware rtc counter. */
static void efm32_rtc_start_counter(struct efm32_rtc_private_s *pv)
{
#ifdef CONFIG_DEVICE_CLOCK_GATING
  dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER_CLOCK);
#endif

#if EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 0
  while (cpu_mem_read_32(EFM32_RTC_ADDR + EFM32_RTC_SYNCBUSY_ADDR) &
         EFM32_RTC_SYNCBUSY_CTRL)
    ;
  cpu_mem_write_32(EFM32_RTC_ADDR + EFM32_RTC_CTRL_ADDR, EFM32_RTC_CTRL_EN(COUNT));

#elif EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 1
  cpu_mem_write_32(EFM32_RTCC_ADDR + EFM32_RTCC_CTRL_ADDR,
        EFM32_RTCC_CTRL_ENABLE | (pv->prescaler << EFM32_RTCC_CTRL_CNTPRESC_SHIFT));
#endif
}

/* This function stops the hardware rtc counter. */
static void efm32_rtc_stop_counter(struct efm32_rtc_private_s *pv)
{
#if EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 0
  while (cpu_mem_read_32(EFM32_RTC_ADDR + EFM32_RTC_SYNCBUSY_ADDR) &
         EFM32_RTC_SYNCBUSY_CTRL)
    ;
  cpu_mem_write_32(EFM32_RTC_ADDR + EFM32_RTC_CTRL_ADDR, 0);

#elif EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 1
  cpu_mem_write_32(EFM32_RTCC_ADDR + EFM32_RTCC_CTRL_ADDR, 0);
#endif

#ifdef CONFIG_DEVICE_CLOCK_GATING
  dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER);
#endif
}

/* This function returns a concatenation of the software rtc value and 
   of the hardware rtc value. If a top value overflow interrupt is pending
   in the rtc, the software rtc value is incremented by one to get the 
   most recent rtc value. */
static uint64_t get_timer_value(struct efm32_rtc_private_s *pv)
{
#if EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 0
  uint64_t value = cpu_mem_read_32(EFM32_RTC_ADDR + EFM32_RTC_CNT_ADDR);
#elif EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 1
  uint64_t value = cpu_mem_read_32(EFM32_RTCC_ADDR + EFM32_RTCC_CNT_ADDR);
#endif

  if (value < EFM32_RTC_HW_MASK / 2)      /* check if a wrap just occured */
    {
# if EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 0
      uint32_t x = cpu_mem_read_32(EFM32_RTC_ADDR + EFM32_RTC_IF_ADDR);
      if (x & EFM32_RTC_IF_OF)
        value += 1ULL << EFM32_RTC_HW_WIDTH;

# elif EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 1
      uint32_t x = cpu_mem_read_32(EFM32_RTCC_ADDR + EFM32_RTCC_IF_ADDR);
      if (x & EFM32_RTCC_IF_OF)
        value += 1ULL << EFM32_RTC_HW_WIDTH;
# endif
    }

  return value + ((uint64_t)pv->swvalue << EFM32_RTC_HW_WIDTH);
}

/* This function disables the interrupt associated to compare/capture
   channel 0. */
static inline void efm32_rtc_disable_compare(struct efm32_rtc_private_s *pv)
{
#if EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 0
  cpu_mem_write_32(EFM32_RTC_ADDR + EFM32_RTC_IEN_ADDR, EFM32_RTC_IEN_OF);

#elif EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 1
  cpu_mem_write_32(EFM32_RTCC_ADDR + EFM32_RTCC_IEN_ADDR, EFM32_RTCC_IEN_OF);
#endif
}

static void efm32_rtc_request_start(struct efm32_rtc_private_s *pv,
                                      struct dev_timer_rq_s *rq,
                                      dev_timer_value_t value)
{
  /* enable hw comparator if software part of the counter match */
  if (((rq->deadline ^ value) & EFM32_RTC_SW_MASK))
    return;

  uint32_t d = rq->deadline & EFM32_RTC_HW_MASK;
  uint32_t s = 0;

#if EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 0

  cpu_mem_write_32(EFM32_RTC_ADDR + EFM32_RTC_IEN_ADDR,
                   EFM32_RTC_IEN_COMP0 | EFM32_RTC_IEN_OF);

  do {
    /* write deadline in Compare */
    while (cpu_mem_read_32(EFM32_RTC_ADDR + EFM32_RTC_SYNCBUSY_ADDR) &
           EFM32_RTC_SYNCBUSY_COMP0)
      ;

    cpu_mem_write_32(EFM32_RTC_ADDR + EFM32_RTC_COMP0_ADDR, d + s);

    uint32_t c = cpu_mem_read_32(EFM32_RTC_ADDR + EFM32_RTC_CNT_ADDR);

    /* hw compare for == only, check for race condition */
    if ((d - c /* LE domain write skew */ - 1) & (1 << (EFM32_RTC_HW_WIDTH - 1)))
      {
        s = s * 2 + 1;
        continue;
      }
  } while (0);

#elif EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 1

  cpu_mem_write_32(EFM32_RTCC_ADDR + EFM32_RTCC_IEN_ADDR,
                   EFM32_RTCC_IEN_CC(0) | EFM32_RTCC_IEN_OF);

  do {
    cpu_mem_write_32(EFM32_RTCC_ADDR + EFM32_RTCC_CC_CCV_ADDR(0), d + s);

    uint32_t c = cpu_mem_read_32(EFM32_RTCC_ADDR + EFM32_RTCC_CNT_ADDR);

    /* hw compare for == only, check for race condition */
    if (((uint64_t)d - c - 1) & (1ULL << (EFM32_RTC_HW_WIDTH - 1)))
      {
        s = s * 2 + 1;
        continue;
      }
  } while (0);
#endif

  logk_trace("rtc skew %u", s);
}

static DEV_IRQ_SRC_PROCESS(efm32_rtc_irq)
{
  struct device_s *dev = ep->base.dev;
  struct efm32_rtc_private_s *pv = dev->drv_pv;
 
  lock_spin(&dev->lock);

#if EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 0
  uint64_t value = cpu_mem_read_32(EFM32_RTC_ADDR + EFM32_RTC_CNT_ADDR);
  uint32_t irq = cpu_mem_read_32(EFM32_RTC_ADDR + EFM32_RTC_IF_ADDR)
    & (EFM32_RTC_IF_COMP0 | EFM32_RTC_IF_OF);

  cpu_mem_write_32(EFM32_RTC_ADDR + EFM32_RTC_IFC_ADDR, irq);

#elif EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 1
  uint64_t value = cpu_mem_read_32(EFM32_RTCC_ADDR + EFM32_RTCC_CNT_ADDR);
  uint32_t irq = cpu_mem_read_32(EFM32_RTCC_ADDR + EFM32_RTCC_IF_ADDR)
    & (EFM32_RTCC_IF_CC(0) | EFM32_RTCC_IF_OF);

  cpu_mem_write_32(EFM32_RTCC_ADDR + EFM32_RTCC_IFC_ADDR, irq);
#endif

  if (irq)
    {
      if (dev->start_count == 0)
        goto err;

#if EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 0
      /* Compare channel interrupt */
      if (irq & EFM32_RTC_IF_COMP0)
        efm32_rtc_disable_compare(pv);

      /* Update the software part of the counter */
      if (irq & EFM32_RTC_IF_OF)
        {

#elif EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 1
      if (irq & EFM32_RTCC_IF_CC(0))
        efm32_rtc_disable_compare(pv);

      /* Update the software part of the counter */
      if (irq & EFM32_RTCC_IF_OF)
        {
#endif
          pv->swvalue++;
          if (value > EFM32_RTC_HW_MASK / 2)      /* wrap just occured */
            value = 0;
        }

      value += (uint64_t)pv->swvalue << EFM32_RTC_HW_WIDTH;

      while (1)
        {
          struct dev_timer_rq_s *rq = dev_timer_rq_head(&pv->queue);
          if (rq == NULL)
            {
              dev->start_count &= ~1;
              if (dev->start_count == 0)
                efm32_rtc_stop_counter(pv);
              break;
            }

          /* setup compare for first request */
          if (rq->deadline > value)
            {
              efm32_rtc_request_start(pv, rq, value);
              break;
            }

          dev_timer_rq_remove(&pv->queue, rq);
          rq->base.drvdata = NULL;

          lock_release(&dev->lock);
          dev_timer_rq_done(rq);
          lock_spin(&dev->lock);
        }

    }

 err:
  lock_release(&dev->lock);
}

static DEV_TIMER_CANCEL(efm32_rtc_cancel)
{
  struct device_s *dev = accessor->dev;
  struct efm32_rtc_private_s *pv = dev->drv_pv;
  error_t err = -ETIMEDOUT;

  LOCK_SPIN_IRQ(&dev->lock);

  if (rq->base.drvdata == pv)
    {
      struct dev_timer_rq_s *rqnext = NULL;
      bool_t first = (dev_timer_rq_prev(&pv->queue, rq) == NULL);

      if (first)
        rqnext = dev_timer_rq_next(&pv->queue, rq);

      dev_timer_rq_remove(&pv->queue, rq);
      rq->base.drvdata = NULL;

      if (first)
        {
          efm32_rtc_disable_compare(pv);

          if (rqnext != NULL)
            {
              /* start next request, raise irq on race condition */
#if EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 0
              cpu_mem_write_32(EFM32_RTC_ADDR + EFM32_RTC_IFC_ADDR, EFM32_RTC_IFC_COMP0);
#elif EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 1
              cpu_mem_write_32(EFM32_RTCC_ADDR + EFM32_RTCC_IFC_ADDR, EFM32_RTCC_IFC_CC(0));
#endif
              efm32_rtc_request_start(pv, rqnext, get_timer_value(pv));
            }
          else
            {
              dev->start_count &= ~1;
              if (dev->start_count == 0)
                efm32_rtc_stop_counter(pv);
            }
        }

      err = 0;
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

#include <mutek/scheduler.h>

static DEV_TIMER_REQUEST(efm32_rtc_request)
{
  struct device_s *dev = accessor->dev;
  struct efm32_rtc_private_s *pv = dev->drv_pv;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

#ifdef CONFIG_DEVICE_CLOCK
  if (rq->rev && rq->rev != pv->rev)
    err = -EAGAIN;
  else
#endif
    {
      /* Start timer if needed */
      if (dev->start_count == 0)
        efm32_rtc_start_counter(pv);

      uint64_t value = get_timer_value(pv);

      if (rq->delay)
        rq->deadline = value + rq->delay;
      else if (rq->deadline <= value)
        {
          err = -ETIMEDOUT;
          goto done;
        }

      dev->start_count |= 1;
      dev_timer_rq_insert(&pv->queue, rq);
      rq->base.drvdata = pv;

      /* start request, raise irq on race condition */
      if (dev_timer_rq_prev(&pv->queue, rq) == NULL)
        {
#if EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 0
          cpu_mem_write_32(EFM32_RTC_ADDR + EFM32_RTC_IFC_ADDR, EFM32_RTC_IFC_COMP0);
#elif EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 1
          cpu_mem_write_32(EFM32_RTCC_ADDR + EFM32_RTCC_IFC_ADDR, EFM32_RTCC_IFC_CC(0));
#endif
          efm32_rtc_request_start(pv, rq, value);
        }

    done:
      if (dev->start_count == 0)
        efm32_rtc_stop_counter(pv);
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEV_USE(efm32_rtc_use)
{
  struct device_accessor_s *accessor = param;

  switch (op)
    {
#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
    case DEV_USE_CLOCK_SINK_FREQ_CHANGED: {
      struct dev_clock_notify_s *chg = param;
      struct dev_clock_sink_ep_s *sink = chg->sink;
      struct device_s *dev = sink->dev;
      struct efm32_rtc_private_s *pv = dev->drv_pv;
      pv->freq = chg->freq;
      pv->rev += 2;
      return 0;
    }
#endif

    case DEV_USE_START: {
      struct device_s *dev = accessor->dev;
      struct efm32_rtc_private_s *pv = dev->drv_pv;
      if (dev->start_count == 0)
        efm32_rtc_start_counter(pv);
      return 0;
    }

    case DEV_USE_STOP: {
      struct device_s *dev = accessor->dev;
      struct efm32_rtc_private_s *pv = dev->drv_pv;
      if (dev->start_count == 0)
        efm32_rtc_stop_counter(pv);
      return 0;
    }

    default:
      return dev_use_generic(param, op);
    }
}

static DEV_TIMER_GET_VALUE(efm32_rtc_get_value)
{
  struct device_s *dev = accessor->dev;
  struct efm32_rtc_private_s *pv = dev->drv_pv;
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

static DEV_TIMER_CONFIG(efm32_rtc_config)
{
  struct device_s *dev = accessor->dev;
  struct efm32_rtc_private_s *pv = dev->drv_pv;
  uint32_t r = 1;

  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (cfg)
    cfg->freq = pv->freq;

#if EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 0
  if (res > 1)
    err = -ERANGE;
#elif EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 1
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
          uint_fast8_t div = res > 32768 ? 15 : bit_msb_index(res);
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
#endif

  if (cfg)
    {
#if defined(CONFIG_DEVICE_CLOCK) || EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 1
      cfg->rev = pv->rev;
#else
      cfg->rev = 1;
#endif
      cfg->cap = pv->cap;
      cfg->res = r;
      cfg->cap |= DEV_TIMER_CAP_STOPPABLE | DEV_TIMER_CAP_HIGHRES | DEV_TIMER_CAP_KEEPVALUE | DEV_TIMER_CAP_TICKLESS;
      cfg->cap |= DEV_TIMER_CAP_REQUEST;
      cfg->max = 0xffffffffffffffffULL;
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

/************************************************************************/


static DEV_INIT(efm32_rtc_init)
{
  struct efm32_rtc_private_s  *pv;

  pv = mem_alloc(sizeof(struct efm32_rtc_private_s), (mem_scope_sys));

  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));
  dev->drv_pv = pv;

  /* enable clock */
  if (dev_drv_clock_init(dev, &pv->clk_ep, 0, DEV_CLOCK_EP_FREQ_NOTIFY |
                     DEV_CLOCK_EP_POWER_CLOCK | DEV_CLOCK_EP_GATING_SYNC, &pv->freq))
    goto err_mem;

#ifdef CONFIG_DEVICE_CLOCK
  pv->rev = 1;

# ifdef CONFIG_DEVICE_CLOCK_VARFREQ
  if (pv->clk_ep.flags & DEV_CLOCK_EP_VARFREQ)
    pv->cap |= DEV_TIMER_CAP_VARFREQ | DEV_TIMER_CAP_CLKSKEW;
# endif
#endif

  device_irq_source_init(dev, &pv->irq_eps, 1, efm32_rtc_irq);

  if (device_irq_source_link(dev, &pv->irq_eps, 1, 1))
    goto err_clk;

  dev_rq_pqueue_init(&pv->queue);

  pv->swvalue = 0;

#if EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 0

  cpu_mem_write_32(EFM32_RTC_ADDR + EFM32_RTC_IFC_ADDR, EFM32_RTC_IFC_MASK);
  cpu_mem_write_32(EFM32_RTC_ADDR + EFM32_RTC_IEN_ADDR, EFM32_RTC_IEN_OF);
  cpu_mem_write_32(EFM32_RTC_ADDR + EFM32_RTC_CTRL_ADDR, 0);

#elif EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 1
  cpu_mem_write_32(EFM32_RTCC_ADDR + EFM32_RTCC_IFC_ADDR, EFM32_RTCC_IFC_MASK);
  cpu_mem_write_32(EFM32_RTCC_ADDR + EFM32_RTCC_IEN_ADDR, EFM32_RTCC_IEN_OF);
  cpu_mem_write_32(EFM32_RTCC_ADDR + EFM32_RTCC_CTRL_ADDR, 0);
  cpu_mem_write_32(EFM32_RTCC_ADDR + EFM32_RTCC_CNT_ADDR, 0);
  cpu_mem_write_32(EFM32_RTCC_ADDR + EFM32_RTCC_CC_CTRL_ADDR(0),
                   EFM32_RTCC_CC_CTRL_MODE_SHIFT_VAL(OUTPUTCOMPARE));
#endif

#ifdef CONFIG_DEVICE_CLOCK_GATING
  dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER);
#endif

  return 0;

 err_clk:
  dev_drv_clock_cleanup(dev, &pv->clk_ep);
 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(efm32_rtc_cleanup)
{
  struct efm32_rtc_private_s *pv = dev->drv_pv;

  dev_drv_clock_cleanup(dev, &pv->clk_ep);

  dev_rq_pqueue_destroy(&pv->queue);

  device_irq_source_unlink(dev, &pv->irq_eps, 1);

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(efm32_rtc_drv, 0, "EFM32 RTC", efm32_rtc,
               DRIVER_TIMER_METHODS(efm32_rtc));

DRIVER_REGISTER(efm32_rtc_drv);

