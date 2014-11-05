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

    Copyright (c) 2014 Julien Peeters <contact@julienpeeters.net>

*/

#include <string.h>

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <hexo/interrupt.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/irq.h>
#include <device/class/pwm.h>
#include <device/class/timer.h>
#include <device/class/iomux.h>

#include <mutek/mem_alloc.h>
#include <mutek/kroutine.h>
#include <mutek/printk.h>

#include <cpp/device/helpers.h>
#include <arch/stm32_rcc.h>
#include <arch/stm32_timer.h>
#include <arch/stm32_memory_map.h>

extern uint32_t stm32f4xx_clock_freq_ahb1;
extern uint32_t stm32f4xx_clock_freq_apb1;
extern uint32_t stm32f4xx_clock_freq_apb2;


#define STM32_PWM_CHANNEL_MAX 4

struct stm32_pwm_private_s
{
  /* PWM/timer base address. */
  uintptr_t                 addr;

  /* PWM channel actual count. */
  uint_fast8_t              count;

  /* PWM hardware counter width. */
  uint_fast8_t              hw_width;

  /* PWM channel selection. */
  struct dev_freq_ratio_s   duty[STM32_PWM_CHANNEL_MAX];
  uint_fast8_t              mask;

  /* PWM cached period. */
  uint32_t                  period;

  /* PWM request queue. */
  struct dev_pwm_rq_queue_s queue;

  /* PWM configuration mode. */
  enum dev_pwm_mode_e       mode;
  struct device_pwm_s       *owner;
};

static
error_t stm32_pwm_duty(struct device_s         *dev,
                       uint_fast8_t            channel,
                       struct dev_freq_ratio_s *duty);

static
error_t stm32_pwm_freq(struct device_s   *dev,
                       struct dev_freq_s *freq)
{
  struct stm32_pwm_private_s *pv = dev->drv_pv;

  /* Get input frequency. */
  struct dev_freq_s freq_res;
  if (device_get_res_freq(dev, &freq_res, 0))
    return -ENOTSUP;

  /* Compute scale factor to the requested frequency. */
  uint64_t scale = freq_res.num * freq->denom / (freq_res.denom * freq->num);
  scale -= scale >> 4;

  /* Compute part of the scale factor the period register cannot handle. */
  uint32_t scale_high = scale >> pv->hw_width;
  if (scale_high & 0xffff0000)
    return -ENOTSUP;

  uint32_t presc, period;
  if (scale_high)
    {
      uint_fast8_t bcnt = sizeof(scale_high) * 8 - __builtin_clz(scale_high);
      presc   = 1     << bcnt;
      period  = scale >> bcnt;
    }
  else
    {
      presc  = 1;
      period = scale;
    }

  /* Save configuration. */
  pv->period = period;

  /* Configure the prescaler. */
  DEVICE_REG_UPDATE_DEV(TIMER, pv->addr, PSC, presc - 1);

  /* Configure the period. */
  DEVICE_REG_UPDATE_DEV(TIMER, pv->addr, ARR, period - 1);

  /* Compute new value of active channels. */
  if (pv->mask)
    {
      uint_fast8_t chan;
      for (chan = 0; chan < pv->count; ++chan)
        {
          if ((pv->mask >> chan) & 0x1)
            stm32_pwm_duty(dev, chan, &pv->duty[chan]);
        }
    }

  return 0;
}

static
error_t stm32_pwm_duty(struct device_s         *dev,
                       uint_fast8_t            channel,
                       struct dev_freq_ratio_s *duty)
{
  struct stm32_pwm_private_s *pv = dev->drv_pv;

  if (duty->num > duty->denom)
    return -ENOTSUP;

  pv->duty[channel] = *duty;

  uint32_t d = duty->num * pv->period / duty->denom;
  DEVICE_REG_IDX_UPDATE_DEV(TIMER, pv->addr, CCR, channel, d);

  return 0;
}

static
void stm32_pwm_polarity(struct device_s         *dev,
                        uint_fast8_t            channel,
                        enum dev_pwm_polarity_e pol)
{
  struct stm32_pwm_private_s *pv = dev->drv_pv;

  if ( pol == DEV_PWM_POL_HIGH )
    DEVICE_REG_FIELD_IDX_UPDATE_DEV(TIMER, pv->addr, CCER, CCP, channel, HIGH);
  else
    DEVICE_REG_FIELD_IDX_UPDATE_DEV(TIMER, pv->addr, CCER, CCP, channel, LOW);
}

static
DEV_PWM_CONFIG(stm32_pwm_config)
{
  struct device_s            *dev = pdev->dev;
  struct stm32_pwm_private_s *pv  = dev->drv_pv;

  uint32_t saved_presc, saved_period;

  error_t err = 0;
  LOCK_SPIN_IRQ(&dev->lock);

  if (pv->mode == DEV_PWM_MODE_FIXED)
    {
      err = -ECANCELED;
      goto cfg_end;
    }

  if (pv->mode == DEV_PWM_MODE_EXCL && pv->owner != pdev)
    {
      err = -EBUSY;
      goto cfg_end;
    }

  saved_presc  = DEVICE_REG_VALUE_DEV(TIMER, pv->addr, PSC);
  saved_period = DEVICE_REG_VALUE_DEV(TIMER, pv->addr, ARR);

  if (cfg->mask & DEV_PWM_MASK_FREQ)
    {
      err = stm32_pwm_freq(dev, &cfg->freq);
      if (err)
        goto cfg_end;
    }

  if (cfg->mask & DEV_PWM_MASK_DUTY)
    {
      err = stm32_pwm_duty(dev, pdev->number, &cfg->duty);
      if (err)
        {
          DEVICE_REG_UPDATE_DEV(TIMER, pv->addr, PSC, saved_presc);
          DEVICE_REG_UPDATE_DEV(TIMER, pv->addr, ARR, saved_period);
          goto cfg_end;
        }
    }

  if (cfg->mask & DEV_PWM_MASK_POL)
    stm32_pwm_polarity(dev, pdev->number, cfg->pol);

  if (cfg->mask & DEV_PWM_MASK_MODE)
    pv->mode = cfg->mode;

cfg_end:
  LOCK_RELEASE_IRQ(&dev->lock);

  cfg->error = err;

  lock_release(&dev->lock);
  kroutine_exec(&cfg->kr, 0);
  lock_spin(&dev->lock);

  return 0;
}

DEV_PWM_QUEUE(stm32_pwm_queue)
{
  struct device_s            *dev = pdev->dev;
  struct stm32_pwm_private_s *pv  = dev->drv_pv;
  return &pv->queue;
}

static
error_t stm32_pwm_start_stop(struct device_s *dev,
                             uint_fast8_t    channel,
                             bool_t          start)
{
  struct stm32_pwm_private_s *pv = dev->drv_pv;

  if (channel > pv->count || pv->period == 0)
    return -EIO;

  LOCK_SPIN_IRQ(&dev->lock);

  if (start)
    {
      /* Enable output OCREF (drive signal to output pin). */
      DEVICE_REG_FIELD_IDX_SET_DEV(TIMER, pv->addr, CCER, CCE, channel);

      if (!pv->mask)
        {
          DEVICE_REG_UPDATE_DEV(TIMER, pv->addr, CNT, 0);
          DEVICE_REG_FIELD_SET_DEV(TIMER, pv->addr, CR1, CEN);
          DEVICE_REG_FIELD_SET_DEV(TIMER, pv->addr, EGR, UG);
        }

      pv->mask |= 1 << channel;
    }
  else 
    {
      /* Disable output OCREF (drive signal to output pin). */
      DEVICE_REG_FIELD_IDX_CLR_DEV(TIMER, pv->addr, CCER, CCE, channel);
      pv->mask &= ~(1 << channel);

      if (!pv->mask)
        DEVICE_REG_FIELD_CLR_DEV(TIMER, pv->addr, CR1, CEN);
    }

  LOCK_RELEASE_IRQ(&dev->lock);
  return 0;
}

static const struct driver_pwm_s stm32_pwm_pwm_drv =
{
  .class_   = DRIVER_CLASS_PWM,
  .f_config = stm32_pwm_config,
  .f_queue  = stm32_pwm_queue,
};

static inline
void stm32_pwm_clock_init(struct device_s *dev)
{
  struct stm32_pwm_private_s *pv = dev->drv_pv;

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

/************************************************************************/

static DEV_INIT(stm32_pwm_init);
static DEV_CLEANUP(stm32_pwm_cleanup);
static DEV_USE(stm32_pwm_use);

const struct driver_s stm32_pwm_drv =
{
  .desc      = "STM32 PWM",
  .f_init    = stm32_pwm_init,
  .f_cleanup = stm32_pwm_cleanup,
  .f_use     = stm32_pwm_use,
  .classes   = {
    &stm32_pwm_pwm_drv,
    0
  }
};

REGISTER_DRIVER(stm32_pwm_drv);

static
DEV_INIT(stm32_pwm_init)
{
  struct stm32_pwm_private_s *pv;
  struct dev_freq_s          res_freq;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  pv = mem_alloc(sizeof(struct stm32_pwm_private_s), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));
  dev->drv_pv = pv;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

  /* Frequency resource. */
  if (device_get_res_freq(dev, &res_freq, 0))
    goto err_mem;

  /* FIXME: Set channel count. */
  switch (pv->addr)
    {
    default: break;

    case STM32_TIM1_ADDR:
    case STM32_TIM2_ADDR:
    case STM32_TIM3_ADDR:
    case STM32_TIM4_ADDR:
    case STM32_TIM5_ADDR:
      pv->count = 4;
      break;

    case STM32_TIM9_ADDR:
      pv->count = 2;
      break;

    case STM32_TIM10_ADDR:
    case STM32_TIM11_ADDR:
      pv->count = 1;
      break;
    }

  /* FIXME: Setup clock gating. */
  stm32_pwm_clock_init(dev);

  /* Stop PWM/timer */
  DEVICE_REG_FIELD_CLR_DEV(TIMER, pv->addr, CR1, CEN);

  /* Configure GPIO. */
  iomux_demux_t loc[STM32_PWM_CHANNEL_MAX];
  if (device_iomux_setup(dev, ">oc1 >oc2? >oc3? >oc4?", loc, NULL, NULL))
    goto err_mem;

  /* FIXME: check iomux declarations against pv->count. */

  /* Setup request queue. */
  dev_pwm_rq_queue_init(&pv->queue);

  /* Check timer width. */
  DEVICE_REG_UPDATE_DEV(TIMER, pv->addr, CNT, 0xffffffff);

  if (DEVICE_REG_VALUE_DEV(TIMER, pv->addr, CNT) & 0xffff0000)
    pv->hw_width = 32;
  else
    pv->hw_width = 16;

  /* Disable interrupts. */
  DEVICE_REG_UPDATE_DEV(TIMER, pv->addr, DIER, 0);

  /* Set capture/icompare channel PWM mode. */
  DEVICE_REG_FIELD_UPDATE_DEV(TIMER, pv->addr, CCMR1OC, OC1M, PWM_MODE_1);

  /* Set capture/compare channel as output. */
  DEVICE_REG_FIELD_UPDATE_DEV(TIMER, pv->addr, CCMR1OC, CC1S, OUTPUT);

  /* Disable CCR preload, updating CCR immediately applies. */
  DEVICE_REG_FIELD_CLR_DEV(TIMER, pv->addr, CCMR1OC, OC1PE);

  /* Set up other channels. */
  if (loc[1] != IOMUX_INVALID_DEMUX && pv->count > 1)
    {
      DEVICE_REG_FIELD_UPDATE_DEV(TIMER, pv->addr, CCMR1OC, OC2M, PWM_MODE_1);
      DEVICE_REG_FIELD_UPDATE_DEV(TIMER, pv->addr, CCMR1OC, CC2S, OUTPUT);
      DEVICE_REG_FIELD_CLR_DEV(TIMER, pv->addr, CCMR1OC, OC2PE);
    }
  if (loc[2] != IOMUX_INVALID_DEMUX && pv->count > 2)
    {
      DEVICE_REG_FIELD_UPDATE_DEV(TIMER, pv->addr, CCMR2OC, OC3M, PWM_MODE_1);
      DEVICE_REG_FIELD_UPDATE_DEV(TIMER, pv->addr, CCMR2OC, CC3S, OUTPUT);
      DEVICE_REG_FIELD_CLR_DEV(TIMER, pv->addr, CCMR2OC, OC3PE);
    }

  if (loc[3] != IOMUX_INVALID_DEMUX && pv->count > 3)
    {
      DEVICE_REG_FIELD_UPDATE_DEV(TIMER, pv->addr, CCMR2OC, OC4M, PWM_MODE_1);
      DEVICE_REG_FIELD_UPDATE_DEV(TIMER, pv->addr, CCMR2OC, CC4S, OUTPUT);
      DEVICE_REG_FIELD_CLR_DEV(TIMER, pv->addr, CCMR2OC, OC4PE);
    }

  /* Upcounting. */
  DEVICE_REG_FIELD_CLR_DEV(TIMER, pv->addr, CR1, DIR);

  /* ster mode and internal clock selection. */
  DEVICE_REG_UPDATE_DEV(TIMER, pv->addr, SMCR, 0);

  /* Set repeat counter to 0, then generate update at each overflow. */
  DEVICE_REG_FIELD_UPDATE_DEV(TIMER, pv->addr, RCR, REP, 0);

  dev->drv    = &stm32_pwm_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;

err_mem:
  mem_free(pv);
  return -1;
}

static
DEV_CLEANUP(stm32_pwm_cleanup)
{
  struct stm32_pwm_private_s *pv = dev->drv_pv;

  DEVICE_REG_FIELD_CLR_DEV(TIMER, pv->addr, CR1, CEN);
  dev_pwm_rq_queue_cleanup(&pv->queue);
  mem_free(pv);
}

static
DEV_USE(stm32_pwm_use)
{
    struct device_s            *dev = acc->dev;
    struct stm32_pwm_private_s *pv  = dev->drv_pv;

    switch (op)
      {
      default:
        return 0;

      case DEV_USE_START:
        return stm32_pwm_start_stop(dev, acc->number, 1);

      case DEV_USE_STOP:
        return stm32_pwm_start_stop(dev, acc->number, 0);

      case DEV_USE_PUT_ACCESSOR:
        if (pv->mode == DEV_PWM_MODE_EXCL && pv->owner == (void*)acc)
          {
            struct dev_pwm_rq_queue_s *q = &pv->queue;
            lock_spin_irq(&q->lock);
            dev_pwm_execute(q);
            lock_release_irq(&q->lock);
          }
        return 0;
      }
}

