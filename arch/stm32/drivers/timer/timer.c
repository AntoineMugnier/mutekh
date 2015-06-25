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

#include <cpp/device/helpers.h>
#include <arch/stm32f4xx_rcc.h>
#include <arch/stm32_timer.h>
#include <arch/stm32_memory_map.h>

#define STM32_TIMER_HW_WIDTH(pv)    (pv)->hw_width
#define STM32_TIMER_HW_MASK(pv)     ((1ULL << pv->hw_width)-1)
#define STM32_TIMER_SW_MASK(pv)     (0xffffffffffffffffULL & ~STM32_TIMER_HW_MASK(pv))
#define STM32_TIMER_TOP(pv)         STM32_TIMER_HW_MASK(pv)
#define STM32_TIMER_CHANNEL         0


struct stm32_timer_private_s
{
  /* Timer address */
  uintptr_t addr;
  /* Start timer counter, bit 0 indicates if there are pending requests */
  uint8_t   hw_width;
#ifdef CONFIG_DEVICE_IRQ
  /* Timer Software value */
  uint64_t swvalue;
  /* Interrupt end-point */
  struct dev_irq_ep_s irq_eps;
  /* Request queue */
  dev_request_pqueue_root_t queue;
#endif

  uint_fast8_t start_count;
  enum dev_timer_capabilities_e cap:8;
  dev_timer_cfgrev_t rev;
};

/* This function starts the hardware timer counter. */
static inline void stm32_timer_start_counter(struct stm32_timer_private_s *pv)
{
    DEVICE_REG_FIELD_SET_DEV(TIMER, pv->addr, CR1, CEN);
}

/* This function stops the hardware timer counter. */
static inline void stm32_timer_stop_counter(struct stm32_timer_private_s *pv)
{
    DEVICE_REG_FIELD_CLR_DEV(TIMER, pv->addr, CR1, CEN);
}

/* This function returns a concatenation of the software timer value and
   of the hardware timer value. If a top value overflow interrupt is pending
   in the timer, the software timer value is incremented by one to get the
   most recent timer value. */
static uint64_t get_timer_value(struct stm32_timer_private_s *pv)
{
  uint64_t value = DEVICE_REG_VALUE_DEV(TIMER, pv->addr, CNT);

#ifdef CONFIG_DEVICE_IRQ
  if (value < STM32_TIMER_HW_MASK(pv) / 2)      /* check if a wrap just occured */
    {
      uint32_t x = DEVICE_REG_FIELD_VALUE_DEV(TIMER, pv->addr, SR, UIF);
      if (x)
        value += 1ULL << STM32_TIMER_HW_WIDTH(pv);
    }

  return value + (pv->swvalue << STM32_TIMER_HW_WIDTH(pv));
#else
  return value;
#endif
}

#ifdef CONFIG_DEVICE_IRQ

/* This function writes a value in the Compare/Capture channel 0 of the
   timer. When the timer counter value will be greater than this value
   a compare interrup will be raised. */
static inline void stm32_timer_enable_compare(struct stm32_timer_private_s *pv, dev_timer_value_t v)
{
  /* Write v in Compare 0 channel */
  DEVICE_REG_IDX_UPDATE_DEV(TIMER, pv->addr, CCR, STM32_TIMER_CHANNEL, v);

  /* Enable interrupt. */
  DEVICE_REG_FIELD_IDX_SET_DEV(TIMER, pv->addr, DIER, CCIE, STM32_TIMER_CHANNEL);
  /* Set output compare active. */
  //DEVICE_REG_FIELD_UPDATE_DEV(TIMER, pv->addr, CCMR1OC, OC1M, ACTIVE);
}

/* This function disables the interrupt associated to compare/capture
   channel 0. */
static inline void stm32_timer_disable_compare(struct stm32_timer_private_s *pv)
{
  /* Disable interrupt. */
  DEVICE_REG_FIELD_IDX_SET_DEV(TIMER, pv->addr, DIER, CCIE, STM32_TIMER_CHANNEL);
  /* Set output compare inactive. */
  //DEVICE_REG_FIELD_UPDATE_DEV(TIMER, pv->addr, CCMR1OC, OC1M, FORCE_INACTIVE);
}

static bool_t stm32_timer_request_start(struct stm32_timer_private_s *pv,
                                        struct dev_timer_rq_s *rq,
                                        dev_timer_value_t value)
{
  /* enable hw comparator if software part of the counter match */
  if ((rq->deadline ^ value) & STM32_TIMER_SW_MASK(pv))
    return 0;

  stm32_timer_enable_compare(pv, rq->deadline);

  /* hw compare for == only, check for race condition */
  if (rq->deadline <= get_timer_value(pv))
    return 1;

  return 0;
}

static inline void stm32_timer_raise_irq(struct stm32_timer_private_s *pv)
{
  DEVICE_REG_FIELD_SET_DEV(TIMER, pv->addr, EGR, UG);
}

static DEV_IRQ_EP_PROCESS(stm32_timer_irq)
{
  struct device_s *dev = ep->dev;
  struct stm32_timer_private_s *pv = dev->drv_pv;

  lock_spin(&dev->lock);

  while (1)
    {
      uint32_t x = DEVICE_REG_VALUE_DEV(TIMER, pv->addr, SR);
      x &= STM32_TIMER_SR_UIF | STM32_TIMER_SR_CCIF(STM32_TIMER_CHANNEL);

      if (!x)
        break;

      DEVICE_REG_UPDATE_DEV(TIMER, pv->addr, SR, 0);

      /* Compare channel interrupt */
      if (x & STM32_TIMER_SR_CCIF(STM32_TIMER_CHANNEL))
      {
        stm32_timer_disable_compare(pv);
      }

      /* Update the software part of the counter */
      if (x & STM32_TIMER_SR_UIF)
        pv->swvalue++;

      while (1)
        {
          struct dev_timer_rq_s *rq;
          rq = dev_timer_rq_s_cast(dev_request_pqueue_head(&pv->queue));
          if (rq == NULL)
            {
              pv->start_count &= ~1;
              if (pv->start_count == 0)
                stm32_timer_stop_counter(pv);
              break;
            }

          uint64_t value = get_timer_value(pv);

          /* setup compare for first request */
          if (rq->deadline > value)
            if (!stm32_timer_request_start(pv, rq, value))
              break;

          dev_timer_pqueue_remove(&pv->queue, dev_timer_rq_s_base(rq));
          stm32_timer_disable_compare(pv);
          rq->rq.drvdata = NULL;

          lock_release(&dev->lock);
          kroutine_exec(&rq->rq.kr, 0);
          lock_spin(&dev->lock);
        }
    }

  lock_release(&dev->lock);
}
#endif

static DEV_TIMER_CANCEL(stm32_timer_cancel)
{
#ifdef CONFIG_DEVICE_IRQ
  struct device_s *dev = accessor->dev;
  struct stm32_timer_private_s *pv = dev->drv_pv;
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
          stm32_timer_disable_compare(pv);

          if (rqnext != NULL)
            {
              /* start next request, raise irq on race condition */
              if (stm32_timer_request_start(pv, rqnext, get_timer_value(pv)))
                stm32_timer_raise_irq(pv);
            }
          else
            {
              pv->start_count &= ~1;
              if (pv->start_count == 0)
                stm32_timer_stop_counter(pv);
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

static DEV_TIMER_REQUEST(stm32_timer_request)
{
#ifdef CONFIG_DEVICE_IRQ
  struct device_s *dev = accessor->dev;
  struct stm32_timer_private_s *pv = dev->drv_pv;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (rq->rev && rq->rev != pv->rev)
    err = -EAGAIN;
  else
    {
      /* Start timer if needed */
      if (pv->start_count == 0)
        stm32_timer_start_counter(pv);

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
            if (stm32_timer_request_start(pv, rq, value))
              stm32_timer_raise_irq(pv);
        }

      if (pv->start_count == 0)
        stm32_timer_stop_counter(pv);
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
#else
  return -ENOTSUP;
#endif
}

static DEV_USE(stm32_timer_use)
{
  struct device_s *dev = accessor->dev;
  struct stm32_timer_private_s *pv = dev->drv_pv;

  error_t err = 0;
  bool_t start = 0;

  switch (op)
    {
    case DEV_USE_GET_ACCESSOR:
    case DEV_USE_PUT_ACCESSOR:
      return 0;
    case DEV_USE_START:
      start = 1;
    case DEV_USE_STOP:
      break;
    }

  LOCK_SPIN_IRQ(&dev->lock);

  if (start)
    {
      if (pv->start_count == 0)
        stm32_timer_start_counter(pv);
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
            stm32_timer_stop_counter(pv);
        }
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEV_TIMER_GET_VALUE(stm32_timer_get_value)
{
  struct device_s *dev = accessor->dev;
  struct stm32_timer_private_s *pv = dev->drv_pv;

  LOCK_SPIN_IRQ(&dev->lock);

  *value = get_timer_value(pv);

  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}

static DEV_TIMER_CONFIG(stm32_timer_config)
{
  struct device_s *dev = accessor->dev;
  struct stm32_timer_private_s *pv = dev->drv_pv;
  uint32_t r;

  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (res)
    {
      if (pv->start_count)
        {
          err = -EBUSY;
          r = res;
        }
      else
        {
          /* check that resolution value is representable in prescaler
           * register. */
          if (res > 0x10000)
            res = 0x10000;

          r = res;
          DEVICE_REG_UPDATE_DEV(TIMER, pv->addr, PSC, r-1);

          /* update configuration revision. */
          pv->rev += 2;
        }
    }
  else
    {
      r = DEVICE_REG_VALUE_DEV(TIMER, pv->addr, PSC) + 1;
    }

  if (cfg)
    {
      // FIXME: hardcoded
      cfg->freq.num   = 84000000;
      cfg->freq.denom = 1;
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

static DEV_INIT(stm32_timer_init);
static DEV_CLEANUP(stm32_timer_cleanup);

DRIVER_DECLARE(stm32_timer_drv, "STM32 Timer", stm32_timer,
               DRIVER_TIMER_METHODS(stm32_timer));

DRIVER_REGISTER(stm32_timer_drv);

static inline
void stm32_timer_clock_init(struct device_s *dev)
{
  struct stm32_timer_private_s *pv = dev->drv_pv;

  assert(pv != 0);

  switch (pv->addr)
    {
    default:
      break;

    case STM32_TIM1_ADDR:
      DEVICE_REG_FIELD_SET(RCC, , APB2ENR, TIM1EN);
      break;

    case STM32_TIM2_ADDR:
      DEVICE_REG_FIELD_SET(RCC, , APB1ENR, TIM2EN);
      break;

    case STM32_TIM3_ADDR:
      DEVICE_REG_FIELD_SET(RCC, , APB1ENR, TIM3EN);
      break;

    case STM32_TIM4_ADDR:
      DEVICE_REG_FIELD_SET(RCC, , APB1ENR, TIM4EN);
      break;

    case STM32_TIM5_ADDR:
      DEVICE_REG_FIELD_SET(RCC, , APB1ENR, TIM5EN);
      break;

    case STM32_TIM9_ADDR:
      DEVICE_REG_FIELD_SET(RCC, , APB2ENR, TIM9EN);
      break;

    case STM32_TIM10_ADDR:
      DEVICE_REG_FIELD_SET(RCC, , APB2ENR, TIM10EN);
      break;

    case STM32_TIM11_ADDR:
      DEVICE_REG_FIELD_SET(RCC, , APB2ENR, TIM11EN);
      break;
    }
}

static DEV_INIT(stm32_timer_init)
{
  struct stm32_timer_private_s  *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  uintptr_t addr;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL))
    return -ENOENT;

  pv = mem_alloc(sizeof(struct stm32_timer_private_s), (mem_scope_sys));

  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));
  pv->addr = addr;
  pv->start_count = 0;
  pv->rev = 1;
  pv->cap = DEV_TIMER_CAP_STOPPABLE | DEV_TIMER_CAP_HIGHRES | DEV_TIMER_CAP_KEEPVALUE;
  dev->drv_pv = pv;

#ifdef CONFIG_DEVICE_IRQ
  pv->cap |= DEV_TIMER_CAP_REQUEST;

  device_irq_source_init(dev, &pv->irq_eps, 1,
                         stm32_timer_irq, DEV_IRQ_SENSE_HIGH_LEVEL);

  if (device_irq_source_link(dev, &pv->irq_eps, 1, 1))
    goto err_clk;

  dev_request_pqueue_init(&pv->queue);
#else
  cfg->cap |= DEV_TIMER_CAP_TICKLESS;
#endif

  /* FIXME: setup clock. */
  stm32_timer_clock_init(dev);

  /* Stop timer */
  DEVICE_REG_FIELD_CLR_DEV(TIMER, pv->addr, CR1, CEN);

  /* Check hw width. */
  DEVICE_REG_UPDATE_DEV(TIMER, pv->addr, ARR, 0xffffffff);
  if ((DEVICE_REG_VALUE_DEV(TIMER, pv->addr, ARR) >> 16) == 0x0)
    pv->hw_width = 16;
  else
    pv->hw_width = 32;

#ifdef CONFIG_DEVICE_IRQ
  /* Clear interrupts */
  DEVICE_REG_UPDATE_DEV(TIMER, pv->addr, SR, 0);

  /* Enable Overflow interrupts */
  DEVICE_REG_FIELD_SET_DEV(TIMER, pv->addr, DIER, UIE);

  /* Set Compare/Capture channel 0 to Output Compare */
  DEVICE_REG_FIELD_UPDATE_DEV(TIMER, pv->addr, CCMR1OC, CC1S, OUTPUT);

  pv->swvalue = 0;
#endif

  /* Prescaler 1MHz. */
  // FIXME: 84MHz hardcoded clock freq.
  DEVICE_REG_UPDATE_DEV(TIMER, pv->addr, PSC, 83);
  DEVICE_REG_FIELD_SET_DEV(TIMER, pv->addr, EGR, UG);

  /* Top value. */
  DEVICE_REG_UPDATE_DEV(TIMER, pv->addr, ARR, STM32_TIMER_TOP(pv));

  /* Disable compare on startup. */
  stm32_timer_disable_compare(pv);

  dev->drv = &stm32_timer_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;

 err_clk:
#ifdef CONFIG_DEVICE_IRQ
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

static DEV_CLEANUP(stm32_timer_cleanup)
{
  struct stm32_timer_private_s *pv = dev->drv_pv;

  /* Stop timer */
  stm32_timer_stop_counter(pv);

#ifdef CONFIG_DEVICE_CLOCK
  dev_clock_sink_release(&pv->clk_ep);
  dev_clock_sink_unlink(dev, &pv->clk_ep, 1);
#endif

#ifdef CONFIG_DEVICE_IRQ
  dev_request_pqueue_destroy(&pv->queue);

  device_irq_source_unlink(dev, &pv->irq_eps, 1);
#endif

  mem_free(pv);
}

