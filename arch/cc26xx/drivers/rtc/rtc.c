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

    Copyright (c) 2015 Vincent Defilippi <vincentdefilippi@gmail.com>

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

#include <mutek/printk.h>
#include <mutek/mem_alloc.h>
#include <mutek/kroutine.h>

#include <arch/cc26xx/memory_map.h>
#include <arch/cc26xx/aon_rtc.h>


#define CC26XX_RTC_HW_WIDTH 32
#define CC26XX_RTC_HW_MASK  0xffffffff
#define CC26XX_RTC_SW_MASK  0xffffffff00000000ULL
#define CC26XX_RTC_TOP      0xffffffff

struct cc26xx_rtc_private_s
{
  /* RTC address */
  uintptr_t addr;
#ifdef CONFIG_DEVICE_IRQ
  /* Timer software value */
  uint64_t swvalue;
  /* Interrupt end-point */
  struct dev_irq_src_s irq_eps;
  /* Request queue */
  dev_request_pqueue_root_t queue;
#endif

  struct dev_freq_s freq;
  struct dev_freq_accuracy_s acc;
  enum dev_timer_capabilities_e cap:8;
  dev_timer_cfgrev_t rev;
};

/* This function starts the hardware rtc */
static inline void cc26xx_rtc_start_counter(struct cc26xx_rtc_private_s *pv)
{
  /* Enable the rtc counter */
  uint32_t reg = cpu_mem_read_32(pv->addr + CC26XX_AON_RTC_CTL_ADDR);
  cpu_mem_write_32(pv->addr + CC26XX_AON_RTC_CTL_ADDR, reg | CC26XX_AON_RTC_CTL_EN);
  cpu_mem_read_32(pv->addr + CC26XX_AON_RTC_SYNC_ADDR);
}

/* This function stops the hardware rtc */
static inline void cc26xx_rtc_stop_counter(struct cc26xx_rtc_private_s *pv)
{
  /* Disable the rtc counter */
  uint32_t reg = cpu_mem_read_32(pv->addr + CC26XX_AON_RTC_CTL_ADDR);
  cpu_mem_write_32(pv->addr + CC26XX_AON_RTC_CTL_ADDR, reg & ~CC26XX_AON_RTC_CTL_EN);
  cpu_mem_read_32(pv->addr + CC26XX_AON_RTC_SYNC_ADDR);
}

/* This function returns a concatenation of the software rtc value and 
   of the hardware rtc value. If a top value overflow interrupt is pending
   in the rtc, the software timer value is incremented by one to get the 
   most recent rtc value. */
static uint64_t get_rtc_value(struct cc26xx_rtc_private_s *pv)
{
  /* The LSB register is automatically latched when the MSB register is read */
  uint32_t msb_val = cpu_mem_read_32(pv->addr + CC26XX_AON_RTC_SEC_ADDR);
  uint32_t lsb_val = cpu_mem_read_32(pv->addr + CC26XX_AON_RTC_SUBSEC_ADDR);

  uint64_t value = (msb_val << 16) + (lsb_val >> 16);

#ifdef CONFIG_DEVICE_IRQ
  if (value < CC26XX_RTC_HW_MASK / 2)      /* check if a wrap just occured */
    {
      uint32_t x = cpu_mem_read_32(pv->addr + CC26XX_AON_RTC_EVFLAGS_ADDR);
      if (x & CC26XX_AON_RTC_EVFLAGS_CH0)
        value += 1ULL << CC26XX_RTC_HW_WIDTH;
    }
  return value + (pv->swvalue << CC26XX_RTC_HW_WIDTH);
#else
  return value;
#endif
}

#ifdef CONFIG_DEVICE_IRQ

/* This function writes a value in the Comparator of the rtc.
   When the rtc value will be greater than this value a compare interrup will be raised. */
static inline void cc26xx_rtc_enable_compare(struct cc26xx_rtc_private_s *pv, dev_timer_value_t v)
{
  /* Write v in Comparator */
  cpu_mem_write_32(pv->addr + CC26XX_AON_RTC_CH1CMP_ADDR, v);

  /* Enable channel 0 */
  uint32_t reg = cpu_mem_read_32(pv->addr + CC26XX_AON_RTC_CHCTL_ADDR);
  cpu_mem_write_32(pv->addr + CC26XX_AON_RTC_CHCTL_ADDR, reg | CC26XX_AON_RTC_CHCTL_CH1_EN);
  cpu_mem_read_32(pv->addr + CC26XX_AON_RTC_SYNC_ADDR);
}


/* This function disables the comparator */
static inline void cc26xx_rtc_disable_compare(struct cc26xx_rtc_private_s *pv)
{
  /* Disable channel 1 */
  uint32_t reg = cpu_mem_read_32(pv->addr + CC26XX_AON_RTC_CHCTL_ADDR);
  cpu_mem_write_32(pv->addr + CC26XX_AON_RTC_CHCTL_ADDR, reg & ~CC26XX_AON_RTC_CHCTL_CH1_EN);
  cpu_mem_read_32(pv->addr + CC26XX_AON_RTC_SYNC_ADDR);
}

static bool_t cc26xx_rtc_request_start(struct cc26xx_rtc_private_s *pv,
                                      struct dev_timer_rq_s *rq,
                                      dev_timer_value_t value)
{
  /* enable hw comparator if software part of the counter match */
  if (((rq->deadline ^ value) & CC26XX_RTC_SW_MASK))
    return 0;

  cc26xx_rtc_enable_compare(pv, rq->deadline);

  /* hw compare for == only, check for race condition */
  if (rq->deadline <= get_rtc_value(pv))
    return 1;

  return 0;
}

static void cc26xx_rtc_rq_handler(struct device_s *dev)
{
  struct cc26xx_rtc_private_s *pv = dev->drv_pv;

  while (1)
    {
      struct dev_timer_rq_s *rq;
      rq = dev_timer_rq_s_cast(dev_request_pqueue_head(&pv->queue));
      if (rq == NULL)
        {
          dev->start_count &= ~1;
          if (dev->start_count == 0)
            cc26xx_rtc_stop_counter(pv);
          break;
        }

      uint64_t value = get_rtc_value(pv);

      /* setup compare for first request */
      if (rq->deadline > value)
        if (!cc26xx_rtc_request_start(pv, rq, value))
          break;

      dev_timer_pqueue_remove(&pv->queue, dev_timer_rq_s_base(rq));
      cc26xx_rtc_disable_compare(pv);
      rq->rq.drvdata = NULL;

      lock_release(&dev->lock);
      kroutine_exec(&rq->rq.kr);
      lock_spin(&dev->lock);
    }

}

static DEV_IRQ_SRC_PROCESS(cc26xx_rtc_irq)
{
  struct device_s *dev = ep->base.dev;
  struct cc26xx_rtc_private_s *pv = dev->drv_pv;

  lock_spin(&dev->lock);

  while (1)
    {
      uint32_t irq = cpu_mem_read_32(pv->addr + CC26XX_AON_RTC_EVFLAGS_ADDR)
                  & (CC26XX_AON_RTC_EVFLAGS_CH0 | CC26XX_AON_RTC_EVFLAGS_CH1);

      if (!irq)
        break;

      /* Clear events flags */
      cpu_mem_write_32(pv->addr + CC26XX_AON_RTC_EVFLAGS_ADDR,
                        CC26XX_AON_RTC_EVFLAGS_CH0 | CC26XX_AON_RTC_EVFLAGS_CH1);

      /* Compare interrupt */
      if (irq & CC26XX_AON_RTC_EVFLAGS_CH1)
        cc26xx_rtc_disable_compare(pv);

      /* Update the software part of the counter */
      if (irq & CC26XX_AON_RTC_EVFLAGS_CH0)
        pv->swvalue++;

      cc26xx_rtc_rq_handler(dev);
    }

  lock_release(&dev->lock);
}
#endif

static DEV_TIMER_CANCEL(cc26xx_rtc_cancel)
{
#ifdef CONFIG_DEVICE_IRQ
  struct device_s *dev = accessor->dev;
  struct cc26xx_rtc_private_s *pv = dev->drv_pv;
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
          cc26xx_rtc_disable_compare(pv);

          if (rqnext != NULL)
            {
              /* start next request, raise irq on race condition */
              if (cc26xx_rtc_request_start(pv, rqnext, get_rtc_value(pv)))
                cc26xx_rtc_rq_handler(dev);
            }
          else
            {
              dev->start_count &= ~1;
              if (dev->start_count == 0)
                cc26xx_rtc_stop_counter(pv);
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

static DEV_TIMER_REQUEST(cc26xx_rtc_request)
{
#ifdef CONFIG_DEVICE_IRQ
  struct device_s *dev = accessor->dev;
  struct cc26xx_rtc_private_s *pv = dev->drv_pv;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (rq->rev && rq->rev != pv->rev)
    err = -EAGAIN;
  else
    {
      /* Start rtc if needed */
      if (dev->start_count == 0)
        cc26xx_rtc_start_counter(pv);

      uint64_t value = get_rtc_value(pv);

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
            if (cc26xx_rtc_request_start(pv, rq, value))
              cc26xx_rtc_rq_handler(dev);
        }

      if (dev->start_count == 0)
        cc26xx_rtc_stop_counter(pv);
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
#else
  return -ENOTSUP;
#endif
}

static DEV_USE(cc26xx_rtc_use)
{
  struct device_accessor_s *accessor = param;

  switch (op)
    {
    case DEV_USE_START: {
      struct device_s *dev = accessor->dev;
      struct cc26xx_rtc_private_s *pv = dev->drv_pv;
      if (dev->start_count == 0)
        cc26xx_rtc_start_counter(pv);
      return 0;
    }

    case DEV_USE_STOP: {
      struct device_s *dev = accessor->dev;
      struct cc26xx_rtc_private_s *pv = dev->drv_pv;
      if (dev->start_count == 0)
        cc26xx_rtc_stop_counter(pv);
      return 0;
    }

    default:
      return dev_use_generic(param, op);
    }
}

static DEV_TIMER_GET_VALUE(cc26xx_rtc_get_value)
{
  struct device_s *dev = accessor->dev;
  struct cc26xx_rtc_private_s *pv = dev->drv_pv;

  LOCK_SPIN_IRQ(&dev->lock);

  *value = get_rtc_value(pv);

  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}

static DEV_TIMER_CONFIG(cc26xx_rtc_config)
{
  struct device_s *dev = accessor->dev;
  struct cc26xx_rtc_private_s *pv = dev->drv_pv;

  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (cfg)
    {
      cfg->rev = pv->rev;
      cfg->res = 1;
      cfg->cap = pv->cap;
      cfg->freq.num = pv->freq.num;
      cfg->freq.denom = pv->freq.denom;
      cfg->acc.e = 0;
#ifdef CONFIG_DEVICE_IRQ
      cfg->max = 0xffffffffffffffffULL;
#else
      cfg->max = 0xffffffff;
#endif
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

/************************************************************************/

static DEV_INIT(cc26xx_rtc_init);
static DEV_CLEANUP(cc26xx_rtc_cleanup);

DRIVER_DECLARE(cc26xx_rtc_drv, 0, "CC26XX RTC", cc26xx_rtc,
               DRIVER_TIMER_METHODS(cc26xx_rtc));

DRIVER_REGISTER(cc26xx_rtc_drv);

static DEV_INIT(cc26xx_rtc_init)
{
  struct cc26xx_rtc_private_s  *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  uintptr_t addr;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL))
    return -ENOENT;

  pv = mem_alloc(sizeof(struct cc26xx_rtc_private_s), (mem_scope_sys));

  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));
  pv->addr = addr;
  pv->rev = 1;
  pv->cap = DEV_TIMER_CAP_STOPPABLE | DEV_TIMER_CAP_HIGHRES | DEV_TIMER_CAP_KEEPVALUE;
  dev->drv_pv = pv;

  if (device_get_res_freq(dev, &pv->freq, 0))
    goto err_mem;

#ifdef CONFIG_DEVICE_IRQ
  pv->cap |= DEV_TIMER_CAP_REQUEST;

  device_irq_source_init(dev, &pv->irq_eps, 1, cc26xx_rtc_irq);

  if (device_irq_source_link(dev, &pv->irq_eps, 1, 1))
    goto err_mem;

  dev_request_pqueue_init(&pv->queue);
#else
  pv->cap |= DEV_TIMER_CAP_TICKLESS;
#endif

  /* disable and reset the rtc */
  cpu_mem_write_32(pv->addr + CC26XX_AON_RTC_CTL_ADDR, CC26XX_AON_RTC_CTL_RESET);

#ifdef CONFIG_DEVICE_IRQ
  /* Clear event flags for channel 0 and channel 1 */
  cpu_mem_write_32(pv->addr + CC26XX_AON_RTC_EVFLAGS_ADDR,
                    CC26XX_AON_RTC_EVFLAGS_CH0 | CC26XX_AON_RTC_EVFLAGS_CH1);

  /* Combine channel 0 event and channel 1 event as cpu RTC interrupt */
  cpu_mem_write_32(pv->addr + CC26XX_AON_RTC_CTL_ADDR,
                    CC26XX_AON_RTC_CTL_COMB_EV_MASK(CH0)
                    | CC26XX_AON_RTC_CTL_COMB_EV_MASK(CH1));

  /* Channel 0 handles overflow */
  /* Load the channel 0 comparator with the max value */
  cpu_mem_write_32(pv->addr + CC26XX_AON_RTC_CH0CMP_ADDR, CC26XX_RTC_TOP);

  /* Enable channel 0 comparator */
  cpu_mem_write_32(pv->addr + CC26XX_AON_RTC_CHCTL_ADDR, CC26XX_AON_RTC_CHCTL_CH0_EN);

  pv->swvalue = 0;
#endif

  cpu_mem_read_32(pv->addr + CC26XX_AON_RTC_SYNC_ADDR);

  dev->drv = &cc26xx_rtc_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;

 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(cc26xx_rtc_cleanup)
{
  struct cc26xx_rtc_private_s *pv = dev->drv_pv;

  /* Stop rtc */
  cc26xx_rtc_stop_counter(pv);

#ifdef CONFIG_DEVICE_IRQ
  dev_request_pqueue_destroy(&pv->queue);

  device_irq_source_unlink(dev, &pv->irq_eps, 1);
#endif

  mem_free(pv);

  return 0;
}

