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
  struct dev_irq_src_s irq_eps;
  /* Request queue */
  dev_request_pqueue_root_t queue;
#endif

  enum dev_timer_capabilities_e cap:8;
  dev_timer_cfgrev_t rev;
};

/* This function starts the hardware timer counter. */
static inline void stm32_timer_start_counter(struct stm32_timer_private_s *pv)
{
    do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_TIMER_CR1_ADDR) ))); STM32_TIMER_CR1_CEN_SET( (_reg), 1 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_TIMER_CR1_ADDR) ), endian_le32(_reg) ); } while (0);
}

/* This function stops the hardware timer counter. */
static inline void stm32_timer_stop_counter(struct stm32_timer_private_s *pv)
{
    do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_TIMER_CR1_ADDR) ))); STM32_TIMER_CR1_CEN_SET( (_reg), 0 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_TIMER_CR1_ADDR) ), endian_le32(_reg) ); } while (0);
}

/* This function returns a concatenation of the software timer value and
   of the hardware timer value. If a top value overflow interrupt is pending
   in the timer, the software timer value is incremented by one to get the
   most recent timer value. */
static uint64_t get_timer_value(struct stm32_timer_private_s *pv)
{
  uint64_t value = endian_le32(cpu_mem_read_32(( (((pv->addr))) + (STM32_TIMER_CNT_ADDR) )));

#ifdef CONFIG_DEVICE_IRQ
  if (value < STM32_TIMER_HW_MASK(pv) / 2) /* check if a wrap just occured */
    {
      uint32_t x = STM32_TIMER_SR_UIF_GET( endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_TIMER_SR_ADDR) ))) );
      if (x)
        value += 1ULL << STM32_TIMER_HW_WIDTH(pv);
    }

  return value + (pv->swvalue << STM32_TIMER_HW_WIDTH(pv));
#else
  return value;
#endif
}

/* This function writes a value in the Compare/Capture channel 0 of the
   timer. When the timer counter value will be greater than this value
   a compare interrup will be raised. */
static inline void stm32_timer_enable_compare(struct stm32_timer_private_s *pv, dev_timer_value_t v)
{
  /* Write v in Compare 0 channel */
  cpu_mem_write_32( ( (((pv->addr))) + (STM32_TIMER_CCR_ADDR(STM32_TIMER_CHANNEL)) ), endian_le32(v) );

  /* Enable interrupt. */
  do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_TIMER_DIER_ADDR) ))); STM32_TIMER_DIER_CCIE_SET( STM32_TIMER_CHANNEL, (_reg), 1 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_TIMER_DIER_ADDR) ), endian_le32(_reg) ); } while (0);
  /* Set output compare active. */
  //DEVICE_REG_FIELD_UPDATE_DEV(TIMER, pv->addr, CCMR1OC, OC1M, ACTIVE);
}

/* This function disables the interrupt associated to compare/capture
   channel 0. */
static inline void stm32_timer_disable_compare(struct stm32_timer_private_s *pv)
{
  /* Disable interrupt. */
  do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_TIMER_DIER_ADDR) ))); STM32_TIMER_DIER_CCIE_SET( STM32_TIMER_CHANNEL, (_reg), 1 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_TIMER_DIER_ADDR) ), endian_le32(_reg) ); } while (0);
  /* Set output compare inactive. */
  //DEVICE_REG_FIELD_UPDATE_DEV(TIMER, pv->addr, CCMR1OC, OC1M, FORCE_INACTIVE);
}

#ifdef CONFIG_DEVICE_IRQ

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
  do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_TIMER_EGR_ADDR) ))); STM32_TIMER_EGR_UG_SET( (_reg), 1 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_TIMER_EGR_ADDR) ), endian_le32(_reg) ); } while (0);
}

static DEV_IRQ_SRC_PROCESS(stm32_timer_irq)
{
  struct device_s *dev = ep->base.dev;
  struct stm32_timer_private_s *pv = dev->drv_pv;

  lock_spin(&dev->lock);

  while (1)
    {
      uint32_t x = endian_le32(cpu_mem_read_32(( (((pv->addr))) + (STM32_TIMER_SR_ADDR) )));
      x &= STM32_TIMER_SR_UIF | STM32_TIMER_SR_CCIF(STM32_TIMER_CHANNEL);

      if (!x)
        break;

      cpu_mem_write_32( ( (((pv->addr))) + (STM32_TIMER_SR_ADDR) ), endian_le32(0) );

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
              dev->start_count &= ~1;
              if (dev->start_count == 0)
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
          kroutine_exec(&rq->rq.kr);
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
              dev->start_count &= ~1;
              if (dev->start_count == 0)
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
      if (dev->start_count == 0)
        stm32_timer_start_counter(pv);

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
            if (stm32_timer_request_start(pv, rq, value))
              stm32_timer_raise_irq(pv);
        }

      if (dev->start_count == 0)
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
  struct device_accessor_s *accessor = param;

  switch (op)
    {
    case DEV_USE_START: {
      struct device_s *dev = accessor->dev;
      struct stm32_timer_private_s *pv = dev->drv_pv;
      if (dev->start_count == 0)
        stm32_timer_start_counter(pv);
      return 0;
    }

    case DEV_USE_STOP: {
      struct device_s *dev = accessor->dev;
      struct stm32_timer_private_s *pv = dev->drv_pv;
      if (dev->start_count == 0)
        stm32_timer_start_counter(pv);
      return 0;
    }

    default:
      return dev_use_generic(param, op);
    }

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
      if (dev->start_count)
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
          cpu_mem_write_32( ( (((pv->addr))) + (STM32_TIMER_PSC_ADDR) ), endian_le32(r-1) );

          /* update configuration revision. */
          pv->rev += 2;
        }
    }
  else
    {
      r = endian_le32(cpu_mem_read_32(( (((pv->addr))) + (STM32_TIMER_PSC_ADDR) ))) + 1;
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

DRIVER_DECLARE(stm32_timer_drv, 0, "STM32 Timer", stm32_timer,
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
      do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((STM32_RCC_ADDR)))) + (STM32_RCC_APB2ENR_ADDR) ))); STM32_RCC_APB2ENR_TIM1EN_SET( (_reg), 1 ); cpu_mem_write_32( ( ((((STM32_RCC_ADDR)))) + (STM32_RCC_APB2ENR_ADDR) ), endian_le32(_reg) ); } while (0);
      break;

    case STM32_TIM2_ADDR:
      do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((STM32_RCC_ADDR)))) + (STM32_RCC_APB1ENR_ADDR) ))); STM32_RCC_APB1ENR_TIM2EN_SET( (_reg), 1 ); cpu_mem_write_32( ( ((((STM32_RCC_ADDR)))) + (STM32_RCC_APB1ENR_ADDR) ), endian_le32(_reg) ); } while (0);
      break;

    case STM32_TIM3_ADDR:
      do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((STM32_RCC_ADDR)))) + (STM32_RCC_APB1ENR_ADDR) ))); STM32_RCC_APB1ENR_TIM3EN_SET( (_reg), 1 ); cpu_mem_write_32( ( ((((STM32_RCC_ADDR)))) + (STM32_RCC_APB1ENR_ADDR) ), endian_le32(_reg) ); } while (0);
      break;

    case STM32_TIM4_ADDR:
      do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((STM32_RCC_ADDR)))) + (STM32_RCC_APB1ENR_ADDR) ))); STM32_RCC_APB1ENR_TIM4EN_SET( (_reg), 1 ); cpu_mem_write_32( ( ((((STM32_RCC_ADDR)))) + (STM32_RCC_APB1ENR_ADDR) ), endian_le32(_reg) ); } while (0);
      break;

    case STM32_TIM5_ADDR:
      do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((STM32_RCC_ADDR)))) + (STM32_RCC_APB1ENR_ADDR) ))); STM32_RCC_APB1ENR_TIM5EN_SET( (_reg), 1 ); cpu_mem_write_32( ( ((((STM32_RCC_ADDR)))) + (STM32_RCC_APB1ENR_ADDR) ), endian_le32(_reg) ); } while (0);
      break;

    case STM32_TIM9_ADDR:
      do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((STM32_RCC_ADDR)))) + (STM32_RCC_APB2ENR_ADDR) ))); STM32_RCC_APB2ENR_TIM9EN_SET( (_reg), 1 ); cpu_mem_write_32( ( ((((STM32_RCC_ADDR)))) + (STM32_RCC_APB2ENR_ADDR) ), endian_le32(_reg) ); } while (0);
      break;

    case STM32_TIM10_ADDR:
      do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((STM32_RCC_ADDR)))) + (STM32_RCC_APB2ENR_ADDR) ))); STM32_RCC_APB2ENR_TIM10EN_SET( (_reg), 1 ); cpu_mem_write_32( ( ((((STM32_RCC_ADDR)))) + (STM32_RCC_APB2ENR_ADDR) ), endian_le32(_reg) ); } while (0);
      break;

    case STM32_TIM11_ADDR:
      do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((STM32_RCC_ADDR)))) + (STM32_RCC_APB2ENR_ADDR) ))); STM32_RCC_APB2ENR_TIM11EN_SET( (_reg), 1 ); cpu_mem_write_32( ( ((((STM32_RCC_ADDR)))) + (STM32_RCC_APB2ENR_ADDR) ), endian_le32(_reg) ); } while (0);
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
  pv->rev = 1;
  pv->cap = DEV_TIMER_CAP_STOPPABLE | DEV_TIMER_CAP_HIGHRES | DEV_TIMER_CAP_KEEPVALUE;
  dev->drv_pv = pv;

#ifdef CONFIG_DEVICE_IRQ
  pv->cap |= DEV_TIMER_CAP_REQUEST;

  device_irq_source_init(dev, &pv->irq_eps, 1, stm32_timer_irq);
  if (device_irq_source_link(dev, &pv->irq_eps, 1, -1))
    goto err_clk;

  dev_request_pqueue_init(&pv->queue);
#else
  pv->cap |= DEV_TIMER_CAP_TICKLESS;
#endif

  /* FIXME: setup clock. */
  stm32_timer_clock_init(dev);

  /* Stop timer */
  do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_TIMER_CR1_ADDR) ))); STM32_TIMER_CR1_CEN_SET( (_reg), 0 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_TIMER_CR1_ADDR) ), endian_le32(_reg) ); } while (0);

  /* Check hw width. */
  cpu_mem_write_32( ( (((pv->addr))) + (STM32_TIMER_ARR_ADDR) ), endian_le32(0xffffffff) );
  if ((endian_le32(cpu_mem_read_32(( (((pv->addr))) + (STM32_TIMER_ARR_ADDR) ))) >> 16) == 0x0)
    pv->hw_width = 16;
  else
    pv->hw_width = 32;

#ifdef CONFIG_DEVICE_IRQ
  /* Clear interrupts */
  cpu_mem_write_32( ( (((pv->addr))) + (STM32_TIMER_SR_ADDR) ), endian_le32(0) );

  /* Enable Overflow interrupts */
  do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_TIMER_DIER_ADDR) ))); STM32_TIMER_DIER_UIE_SET( (_reg), 1 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_TIMER_DIER_ADDR) ), endian_le32(_reg) ); } while (0);

  /* Set Compare/Capture channel 0 to Output Compare */
  do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_TIMER_CCMR1OC_ADDR) ))); STM32_TIMER_CCMR1OC_CC1S_SET( (_reg), OUTPUT ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_TIMER_CCMR1OC_ADDR) ), endian_le32(_reg) ); } while (0);

  pv->swvalue = 0;
#endif

  /* Prescaler 1MHz. */
  // FIXME: 84MHz hardcoded clock freq.
  cpu_mem_write_32( ( (((pv->addr))) + (STM32_TIMER_PSC_ADDR) ), endian_le32(83) );
  do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_TIMER_EGR_ADDR) ))); STM32_TIMER_EGR_UG_SET( (_reg), 1 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_TIMER_EGR_ADDR) ), endian_le32(_reg) ); } while (0);

  /* Top value. */
  cpu_mem_write_32( ( (((pv->addr))) + (STM32_TIMER_ARR_ADDR) ), endian_le32(STM32_TIMER_TOP(pv)) );

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

  return 0;
}

