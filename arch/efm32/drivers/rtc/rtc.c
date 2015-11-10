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
#include <device/class/clock.h>

#include <mutek/mem_alloc.h>
#include <mutek/kroutine.h>

#include <arch/efm32_rtc.h>

#define EFM32_RTC_HW_WIDTH 24
#define EFM32_RTC_HW_MASK  0xffffff
#define EFM32_RTC_SW_MASK  0xffffffffff000000ULL

struct efm32_rtc_private_s
{
  /* Timer address */
  uintptr_t addr;
  /* Start timer counter, bit 0 indicates if there are pending requests */
  uint_fast8_t start_count;
  dev_timer_cfgrev_t rev;
#ifdef CONFIG_DEVICE_IRQ
  /* Timer Software value */
  uint32_t swvalue;
  /* Interrupt end-point */
  struct dev_irq_src_s irq_eps;
  /* Request queue */
  dev_request_pqueue_root_t queue;
#endif

#ifdef CONFIG_DEVICE_CLOCK
  struct dev_clock_sink_ep_s clk_ep;
  struct dev_freq_s freq;
  struct dev_freq_accuracy_s acc;
#endif

  enum dev_timer_capabilities_e cap:8;
};

#ifdef CONFIG_DEVICE_CLOCK
static DEV_CLOCK_SINK_CHANGED(efm32_rtc_clk_changed)
{
  struct device_s *dev = ep->dev;
  struct efm32_rtc_private_s *pv = ep->dev->drv_pv;
  LOCK_SPIN_IRQ(&dev->lock);
  pv->rev += 2;
  pv->freq = *freq;
  pv->acc = *acc;
  LOCK_RELEASE_IRQ(&dev->lock);
}
#endif

/* This function starts the hardware rtc counter. */
static inline void efm32_rtc_start_counter(struct efm32_rtc_private_s *pv)
{
  cpu_mem_write_32(pv->addr + EFM32_RTC_CTRL_ADDR, endian_le32(EFM32_RTC_CTRL_EN(COUNT)));
}

/* This function stops the hardware rtc counter. */
static inline void efm32_rtc_stop_counter(struct efm32_rtc_private_s *pv)
{
  cpu_mem_write_32(pv->addr + EFM32_RTC_CTRL_ADDR, 0);
}

/* This function returns a concatenation of the software rtc value and 
   of the hardware rtc value. If a top value overflow interrupt is pending
   in the rtc, the software rtc value is incremented by one to get the 
   most recent rtc value. */
static uint64_t get_timer_value(struct efm32_rtc_private_s *pv)
{
  uint64_t value = endian_le32(cpu_mem_read_32(pv->addr + EFM32_RTC_CNT_ADDR));

#ifdef CONFIG_DEVICE_IRQ
  if (value < EFM32_RTC_HW_MASK / 2)      /* check if a wrap just occured */
    {
      uint32_t x = endian_le32(cpu_mem_read_32(pv->addr + EFM32_RTC_IF_ADDR));
      if (x & EFM32_RTC_IF_OF)
        value += 1ULL << EFM32_RTC_HW_WIDTH;
    }

  return value + ((uint64_t)pv->swvalue << EFM32_RTC_HW_WIDTH);
#else
  return value;
#endif
}

#ifdef CONFIG_DEVICE_IRQ

/* This function writes a value in the Compare channel 0 of the 
   rtc. When the rtc counter value will be greater than this value 
   a compare interrup will be raised. */
static inline void efm32_rtc_enable_compare(struct efm32_rtc_private_s *pv, dev_timer_value_t v)
{
  /* Write v in Compare 0 channel */
  cpu_mem_write_32(pv->addr + EFM32_RTC_COMP0_ADDR, endian_le32(v & EFM32_RTC_COMP0_MASK));

  /* Clear previous pending interrupts */
  cpu_mem_write_32(pv->addr + EFM32_RTC_IFC_ADDR, endian_le32(EFM32_RTC_IFC_COMP0));

  /* Enable Compare/Capture ch 0 interrupts */
  cpu_mem_write_32(pv->addr + EFM32_RTC_IEN_ADDR, endian_le32(EFM32_RTC_IEN_COMP0 | EFM32_RTC_IF_OF));
}

/* This function disables the interruption associated to compare/capture
   channel 0. */
static inline void efm32_rtc_disable_compare(struct efm32_rtc_private_s *pv)
{
  cpu_mem_write_32(pv->addr + EFM32_RTC_IEN_ADDR, endian_le32(EFM32_RTC_IF_OF));
}

static bool_t efm32_rtc_request_start(struct efm32_rtc_private_s *pv,
                                    struct dev_timer_rq_s *rq,
                                    dev_timer_value_t value)
{
  /* enable hw comparator if software part of the counter match */
  if (((rq->deadline ^ value) & EFM32_RTC_SW_MASK))
    return 0;

  efm32_rtc_enable_compare(pv, rq->deadline);

  /* hw compare for == only, check for race condition */
  if (rq->deadline <= get_timer_value(pv))
    return 1;

  return 0;
}

static inline void efm32_rtc_raise_irq(struct efm32_rtc_private_s *pv)
{
  cpu_mem_write_32(pv->addr + EFM32_RTC_IFS_ADDR, endian_le32(EFM32_RTC_IFC_COMP0));
}

static DEV_IRQ_SRC_PROCESS(efm32_rtc_irq)
{
  struct device_s *dev = ep->base.dev;
  struct efm32_rtc_private_s *pv = dev->drv_pv;
 
  lock_spin(&dev->lock);

  while (1)
    {
      uint32_t irq = endian_le32(cpu_mem_read_32(pv->addr + EFM32_RTC_IF_ADDR))
        & (EFM32_RTC_IEN_COMP0 | EFM32_RTC_IF_OF);

      if (!irq)
        break;

      cpu_mem_write_32(pv->addr + EFM32_RTC_IFC_ADDR, endian_le32(irq));

      /* Compare channel interrupt */
      if (irq & EFM32_RTC_IF_COMP0)
        efm32_rtc_disable_compare(pv);

      /* Update the software part of the counter */
      if (irq & EFM32_RTC_IF_OF)
        pv->swvalue++;

      while (1)
        {
          struct dev_timer_rq_s *rq;
          rq = dev_timer_rq_s_cast(dev_request_pqueue_head(&pv->queue));
          if (rq == NULL)
            {
              pv->start_count &= ~1;
              if (pv->start_count == 0)
                efm32_rtc_stop_counter(pv);
              break;
            }

          uint64_t value = get_timer_value(pv);

          /* setup compare for first request */
          if (rq->deadline > value)
            if (!efm32_rtc_request_start(pv, rq, value))
              break;

          dev_timer_pqueue_remove(&pv->queue, dev_timer_rq_s_base(rq));
          efm32_rtc_disable_compare(pv);
          rq->rq.drvdata = NULL;

          lock_release(&dev->lock);
          kroutine_exec(&rq->rq.kr);
          lock_spin(&dev->lock);
        }

    }

  lock_release(&dev->lock);
}
#endif

static DEV_TIMER_CANCEL(efm32_rtc_cancel)
{
#ifdef CONFIG_DEVICE_IRQ
  struct device_s *dev = accessor->dev;
  struct efm32_rtc_private_s *pv = dev->drv_pv;
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
          efm32_rtc_disable_compare(pv);

          if (rqnext != NULL)
            {
              /* start next request, raise irq on race condition */
              if (efm32_rtc_request_start(pv, rqnext, get_timer_value(pv)))
                efm32_rtc_raise_irq(pv);
            }
          else
            {
              pv->start_count &= ~1;
              if (pv->start_count == 0)
                efm32_rtc_stop_counter(pv);
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

#include <mutek/scheduler.h>

static DEV_TIMER_REQUEST(efm32_rtc_request)
{
#ifdef CONFIG_DEVICE_IRQ
  struct device_s *dev = accessor->dev;
  struct efm32_rtc_private_s *pv = dev->drv_pv;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (rq->rev && rq->rev != pv->rev)
    err = -EAGAIN;
  else
    {
      /* Start timer if needed */
      if (pv->start_count == 0)
        efm32_rtc_start_counter(pv);

      uint64_t value = get_timer_value(pv);

      if (rq->delay)
        rq->deadline = value + rq->delay;

      if (rq->deadline <= value)
        err = -ETIMEDOUT;
      else
        {
          pv->start_count |= 1;
          dev_timer_pqueue_insert(&pv->queue, dev_timer_rq_s_base(rq));
          rq->rq.drvdata = pv;

          /* start request, raise irq on race condition */
          if (dev_request_pqueue_prev(&pv->queue, dev_timer_rq_s_base(rq)) == NULL)
            if (efm32_rtc_request_start(pv, rq, value))
              efm32_rtc_raise_irq(pv);
        }

      if (pv->start_count == 0)
        efm32_rtc_stop_counter(pv);
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
#else
  return -ENOTSUP;
#endif
}

static DEV_USE(efm32_rtc_use)
{
  struct device_accessor_s *accessor = param;

  error_t err = 0;

  switch (op)
    {
    case DEV_USE_GET_ACCESSOR:
      if (accessor->number)
        return -ENOTSUP;
    case DEV_USE_PUT_ACCESSOR:
      return 0;
    case DEV_USE_START:
    case DEV_USE_STOP:
      break;
    default:
      return -ENOTSUP;
    }

  struct device_s *dev = accessor->dev;
  struct efm32_rtc_private_s *pv = dev->drv_pv;

  LOCK_SPIN_IRQ(&dev->lock);

  if (op == DEV_USE_START)
    {
      if (pv->start_count == 0)
        efm32_rtc_start_counter(pv);
      pv->start_count += 2;
    }
  else   /* DEV_USE_STOP */
    {
      if (pv->start_count < 2)
        err = -EINVAL;
      else
        {
          pv->start_count -= 2;
          if (pv->start_count == 0)
            efm32_rtc_stop_counter(pv);
        }
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEV_TIMER_GET_VALUE(efm32_rtc_get_value)
{
  struct device_s *dev = accessor->dev;
  struct efm32_rtc_private_s *pv = dev->drv_pv;

  LOCK_SPIN_IRQ(&dev->lock);

  *value = get_timer_value(pv);

  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}

static DEV_TIMER_CONFIG(efm32_rtc_config)
{
  struct device_s *dev = accessor->dev;
  struct efm32_rtc_private_s *pv = dev->drv_pv;

  error_t err = 0;

#ifndef CONFIG_DEVICE_CLOCK
  if (cfg)
    {
      if (device_get_res_freq(accessor->dev, &cfg->freq, 0))
        cfg->freq = DEV_FREQ_INVALID;
      cfg->acc = DEV_FREQ_ACC_INVALID;
    }
#endif

  LOCK_SPIN_IRQ(&dev->lock);

#ifdef CONFIG_DEVICE_CLOCK
  if (cfg)
    {
      cfg->freq = pv->freq;
      cfg->acc = pv->acc;
    }
#endif

  if (res > 1)
    err = -ERANGE;

  if (cfg)
    {
#ifdef CONFIG_DEVICE_CLOCK
      cfg->rev = pv->rev;
#else
      cfg->rev = 1;
#endif
      cfg->cap = pv->cap;
      cfg->res = 1;
      cfg->cap |= DEV_TIMER_CAP_STOPPABLE | DEV_TIMER_CAP_HIGHRES | DEV_TIMER_CAP_KEEPVALUE | DEV_TIMER_CAP_TICKLESS;
#ifdef CONFIG_DEVICE_IRQ
      cfg->cap |= DEV_TIMER_CAP_REQUEST;
      cfg->max = 0xffffffffffffffffULL;
#else
      cfg->max = EFM32_RTC_HW_MASK;
#endif
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

/************************************************************************/

static DEV_INIT(efm32_rtc_init);
static DEV_CLEANUP(efm32_rtc_cleanup);

DRIVER_DECLARE(efm32_rtc_drv, 0, "EFM32 RTC", efm32_rtc,
               DRIVER_TIMER_METHODS(efm32_rtc));

DRIVER_REGISTER(efm32_rtc_drv);

static DEV_INIT(efm32_rtc_init)
{
  struct efm32_rtc_private_s  *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  uintptr_t addr;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL))
    return -ENOENT;

  pv = mem_alloc(sizeof(struct efm32_rtc_private_s), (mem_scope_sys));

  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));
  pv->addr = addr;
  pv->start_count = 0;
  dev->drv_pv = pv;

#ifdef CONFIG_DEVICE_CLOCK
  /* enable clock */
  dev_clock_sink_init(dev, &pv->clk_ep, &efm32_rtc_clk_changed);

  struct dev_clock_link_info_s ckinfo;
  if (dev_clock_sink_link(dev, &pv->clk_ep, &ckinfo, 0, 0))
    goto err_mem;

  if (!DEV_FREQ_IS_VALID(ckinfo.freq))
    goto err_mem;
  pv->freq = ckinfo.freq;
  pv->acc = ckinfo.acc;

  if (ckinfo.src_flags & DEV_CLOCK_SRC_EP_VARFREQ)
    pv->cap |= DEV_TIMER_CAP_VARFREQ | DEV_TIMER_CAP_CLKSKEW;

  if (dev_clock_sink_hold(&pv->clk_ep, 0))
    goto err_clku;

  pv->rev = 1;
#endif

#ifdef CONFIG_DEVICE_IRQ
  device_irq_source_init(dev, &pv->irq_eps, 1, efm32_rtc_irq);

  if (device_irq_source_link(dev, &pv->irq_eps, 1, 1))
    goto err_clk;

  dev_request_pqueue_init(&pv->queue);
#endif

#ifdef CONFIG_DEVICE_IRQ
  /* Clear interrupts */
  cpu_mem_write_32(pv->addr + EFM32_RTC_IFC_ADDR, endian_le32(EFM32_RTC_IFC_MASK));

  /* Enable Overflow interrupts */
  cpu_mem_write_32(pv->addr + EFM32_RTC_IEN_ADDR, endian_le32(EFM32_RTC_IEN_OF));

  pv->swvalue = 0;
#else
  cpu_mem_write_32(pv->addr + EFM32_RTC_IEN_ADDR, 0);
#endif

  /* Ctrl register configuration */
  cpu_mem_write_32(pv->addr + EFM32_RTC_CTRL_ADDR, endian_le32(EFM32_RTC_CTRL_DEBUGRUN(FROZEN) |
                                                               EFM32_RTC_CTRL_EN(RESET) |
                                                               EFM32_RTC_CTRL_COMP0TOP(TOPMAX)));

  dev->drv = &efm32_rtc_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;

#ifdef CONFIG_DEVICE_IRQ
 err_clk:
# ifdef CONFIG_DEVICE_CLOCK
  dev_clock_sink_release(&pv->clk_ep);
# endif
#endif
 err_clku:
#ifdef CONFIG_DEVICE_CLOCK
  dev_clock_sink_unlink(dev, &pv->clk_ep, 1);
#endif
 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(efm32_rtc_cleanup)
{
  struct efm32_rtc_private_s *pv = dev->drv_pv;

  if (pv->start_count & 1)
    return -EBUSY;

  /* Stop rtc */ 
  efm32_rtc_stop_counter(pv);

#ifdef CONFIG_DEVICE_CLOCK
  dev_clock_sink_release(&pv->clk_ep);
  dev_clock_sink_unlink(dev, &pv->clk_ep, 1);
#endif

#ifdef CONFIG_DEVICE_IRQ
  dev_request_pqueue_destroy(&pv->queue);

  device_irq_source_unlink(dev, &pv->irq_eps, 1);
#endif

  mem_free(pv);

  return 0;
}

