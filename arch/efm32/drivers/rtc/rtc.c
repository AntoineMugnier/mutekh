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
  /* Timer Software value */
#ifdef CONFIG_DEVICE_IRQ
  uint32_t swvalue;
  /* Interrupt end-point */
  struct dev_irq_ep_s irq_eps;
  /* Request queue */
  dev_timer_queue_root_t queue;
#endif

#ifdef CONFIG_DEVICE_CLOCK
  struct dev_clock_sink_ep_s clk_ep;
  struct dev_freq_s freq;
  struct dev_freq_accuracy_s acc;
#endif
};

#ifdef CONFIG_DEVICE_CLOCK
static DEV_CLOCK_SINK_CHANGED(efm32_rtc_clk_changed)
{
  struct device_s *dev = ep->dev;
  struct efm32_rtc_private_s *pv = ep->dev->drv_pv;
  LOCK_SPIN_IRQ(&dev->lock);
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

  return value + (pv->swvalue << EFM32_RTC_HW_WIDTH);
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

static DEV_IRQ_EP_PROCESS(efm32_rtc_irq)
{
  struct device_s *dev = ep->dev;
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

      struct dev_timer_rq_s *rq = dev_timer_queue_head(&pv->queue);

      while (rq != NULL)
        {
          uint64_t value = get_timer_value(pv);

          /* setup compare for first request */
          if (rq->deadline > value)
            if (!efm32_rtc_request_start(pv, rq, value))
              break;

          dev_timer_queue_pop(&pv->queue);
          efm32_rtc_disable_compare(pv);
          rq->drvdata = 0;

          lock_release(&dev->lock);
          kroutine_exec(&rq->kr, 0);
          lock_spin(&dev->lock);

          rq = dev_timer_queue_head(&pv->queue);
          if (rq == NULL)
            {
              pv->start_count &= ~1;
              if (pv->start_count == 0)
                efm32_rtc_stop_counter(pv);
              break;
            }
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

  assert(rq->accessor == accessor);

  LOCK_SPIN_IRQ(&dev->lock);

  if (rq->drvdata == pv)
    {
      struct dev_timer_rq_s *rq0 = dev_timer_queue_head(&pv->queue);
     
      dev_timer_queue_remove(&pv->queue, rq);
      rq->drvdata = NULL;

      if (rq == rq0)         /* removed first request ? */
        {
          efm32_rtc_disable_compare(pv);
          rq0 = dev_timer_queue_head(&pv->queue);

          if (rq0 != NULL)
            {
              /* start next request, raise irq on race condition */
              if (efm32_rtc_request_start(pv, rq0, get_timer_value(pv)))
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

  rq->accessor = accessor;

  LOCK_SPIN_IRQ(&dev->lock);

  /* Start timer if needed */
  if (pv->start_count == 0)
    efm32_rtc_start_counter(pv);

  uint64_t value = get_timer_value(pv);

  if (rq->delay)
    rq->deadline = value + rq->delay;

  //  printk("%p %llu %llu\n", sched_get_current(), rq->deadline, value);

  if (rq->deadline <= value)
    err = -ETIMEDOUT;
  else
    {
      pv->start_count |= 1;
      dev_timer_queue_insert(&pv->queue, rq);
      rq->drvdata = pv;

      /* start request, raise irq on race condition */
      if (dev_timer_queue_head(&pv->queue) == rq)
        if (efm32_rtc_request_start(pv, rq, value))
          efm32_rtc_raise_irq(pv);
    }

  if (pv->start_count == 0)
    efm32_rtc_stop_counter(pv);

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
#else
  return -ENOTSUP;
#endif
}

static DEV_TIMER_START_STOP(efm32_rtc_state_start_stop)
{
  struct device_s *dev = accessor->dev;
  struct efm32_rtc_private_s *pv = dev->drv_pv;

  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (start)
    {
      if (pv->start_count == 0)
        efm32_rtc_start_counter(pv);
      pv->start_count += 2;
    }
  else
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

#ifdef CONFIG_DEVICE_CLOCK
static DEV_TIMER_GET_FREQ(efm32_rtc_get_freq)
{
  struct device_s *dev = accessor->dev;
  struct efm32_rtc_private_s *pv = dev->drv_pv;

  *freq = pv->freq;
  return 0;
}
#endif

static DEV_TIMER_RESOLUTION(efm32_rtc_resolution)
{
  struct device_s *dev = accessor->dev;

  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (res)
    {
      if (*res != 0)
        err = -ENOTSUP;
      *res = 1;
    }

  if (max)
#ifdef CONFIG_DEVICE_IRQ
    *max = 0xffffffffffffffffULL;
#else
    *max = 0xffffff;
#endif

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

const struct driver_timer_s efm32_rtc_timer_drv =
{
  .class_         = DRIVER_CLASS_TIMER,
  .f_request      = efm32_rtc_request,
  .f_cancel       = efm32_rtc_cancel,
  .f_start_stop   = efm32_rtc_state_start_stop,
  .f_get_value    = efm32_rtc_get_value,
#ifdef CONFIG_DEVICE_CLOCK
  .f_get_freq     = efm32_rtc_get_freq,
#else
  .f_get_freq     = dev_timer_drv_get_freq,
#endif
  .f_resolution   = efm32_rtc_resolution,
};

/************************************************************************/

static DEV_INIT(efm32_rtc_init);
static DEV_CLEANUP(efm32_rtc_cleanup);

const struct driver_s efm32_rtc_drv =
{
  .desc           = "EFM32 RTC",
  .f_init         = efm32_rtc_init,
  .f_cleanup      = efm32_rtc_cleanup,

  .classes        = {
    &efm32_rtc_timer_drv,
    0
  }
};


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

  if (dev_clock_sink_hold(&pv->clk_ep, NULL))
    goto err_clku;
#endif

#ifdef CONFIG_DEVICE_IRQ
  device_irq_source_init(dev, &pv->irq_eps, 1,
                         efm32_rtc_irq, DEV_IRQ_SENSE_HIGH_LEVEL);

  if (device_irq_source_link(dev, &pv->irq_eps, 1, 1))
    goto err_clk;

  dev_timer_queue_init(&pv->queue);
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

  /* Stop rtc */ 
  efm32_rtc_stop_counter(pv);

#ifdef CONFIG_DEVICE_CLOCK
  dev_clock_sink_release(&pv->clk_ep);
  dev_clock_sink_unlink(dev, &pv->clk_ep, 1);
#endif

#ifdef CONFIG_DEVICE_IRQ
  dev_timer_queue_destroy(&pv->queue);

  device_irq_source_unlink(dev, &pv->irq_eps, 1);
#endif

  mem_free(pv);
}

REGISTER_DRIVER(efm32_rtc_drv);

