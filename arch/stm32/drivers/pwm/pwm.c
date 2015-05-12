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

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
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


#define STM32_PWM_CHANNEL_MAX   4
#define STM32_PWM_CHANNEL_MASK  ((1<<STM32_PWM_CHANNEL_MAX)-1)

struct stm32_pwm_private_s
{
  /* PWM/timer base address. */
  uintptr_t                  addr;

  /* PWM channel flags. */
  uint8_t                    count[STM32_PWM_CHANNEL_MAX];
  uint8_t                    start;
  uint8_t                    config;

  /* PWM hardware counter width. */
  uint8_t                    hw_width;
  uint8_t                    chan_count;

  /* PWM channel selection. */
  struct dev_freq_s          core_freq;
  struct dev_freq_s          freq;
  struct dev_freq_ratio_s    duty[STM32_PWM_CHANNEL_MAX];

  /* PWM cached period. */
  uint32_t                   period;
};

static
error_t stm32_pwm_validate(struct device_pwm_s *pdev,
                           struct dev_pwm_rq_s *rq)
{
  struct stm32_pwm_private_s *pv   = pdev->dev->drv_pv;

  /* A channel is configured before being started. */
  if ((rq->chan_mask << pdev->number) & ~pv->start)
    return -EINVAL;

  struct dev_freq_s const *freq = &rq->cfg[0]->freq;

  uint8_t freq_mask = 0;
  uint8_t const max = STM32_PWM_CHANNEL_MAX - pdev->number;
  uint8_t ci;
  for (ci = 0; ci < max; ++ci)
    {
      if (!(rq->chan_mask & (1 << ci)))
        continue;

      const struct dev_pwm_config_s *cfg = rq->cfg[ci];
      if (cfg->mask & DEV_PWM_MASK_FREQ)
        {
          freq_mask |= 1 << ci;

          /* check shared parameter (frequency). */
          if ((cfg->freq.num != freq->num) || (cfg->freq.denom != freq->denom))
            return -ENOTSUP;
        }
    }

  if (freq_mask)
    {
      /* check shard parameter (frequency). */
      if (freq_mask != rq->chan_mask)
        return -ENOTSUP;
      /* check device are started before configuration. */
      if ((freq_mask << pdev->number) != pv->start)
        return -ENOTSUP;
    }

  return 0;
}

static
error_t stm32_pwm_freq(struct device_s *dev)
{
  struct stm32_pwm_private_s *pv = dev->drv_pv;

  /* Compute scale factor to the requested frequency. */
  uint64_t scale = pv->core_freq.num * pv->freq.denom /
    (pv->core_freq.denom * pv->freq.num);

  /* compute prescaler. */
  uint32_t const presc = scale >> pv->hw_width;

  /* if the divider (in log2) is greater than the prescaler, the
     frequency cannot be achieved.
   */
  if (presc & 0xffff0000)
    return -ERANGE;

  /* compute lower part of the frequency scaling. */
  uint32_t const period = scale / (presc+1);

  /* save the configuration. */
  DEVICE_REG_UPDATE_DEV(TIMER, pv->addr, RCR, 0);
  DEVICE_REG_FIELD_UPDATE_DEV(TIMER, pv->addr, CR1, CKD, NO_DIV);
  DEVICE_REG_UPDATE_DEV(TIMER, pv->addr, PSC, presc);
  DEVICE_REG_UPDATE_DEV(TIMER, pv->addr, ARR, period);

  return 0;
}

static
error_t stm32_pwm_duty(struct device_s *dev, uint_fast8_t channel)
{
  struct stm32_pwm_private_s *pv = dev->drv_pv;

  if (pv->duty[channel].num > pv->duty[channel].denom)
    return -ERANGE;

  uint32_t const period = DEVICE_REG_VALUE_DEV(TIMER, pv->addr, ARR);
  uint32_t const ratio  = pv->duty[channel].num * period / pv->duty[channel].denom;
  DEVICE_REG_IDX_UPDATE_DEV(TIMER, pv->addr, CCR, channel, ratio);

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

  uint_fast8_t ci;
  bool_t       fdone = 0;
  bool_t       start = !(pv->config & STM32_PWM_CHANNEL_MASK);

  error_t err = 0;
  LOCK_SPIN_IRQ(&dev->lock);

  err = stm32_pwm_validate(pdev, rq);
  if (err)
    goto cfg_end;

  uint8_t max = STM32_PWM_CHANNEL_MAX - pdev->number;
  for (ci = 0; ci < max; ++ci)
    {
      if (!(rq->chan_mask & (1 << ci)))
        continue;

      const struct dev_pwm_config_s *cfg = rq->cfg[ci];
      if ((cfg->mask & DEV_PWM_MASK_FREQ) && !fdone)
        {
          pv->freq.num   = cfg->freq.num;
          pv->freq.denom = cfg->freq.denom;

          err = stm32_pwm_freq(dev);
          if (err)
            goto cfg_end;

          fdone = 1;
        }

      uint8_t channel = pdev->number + ci;
      if (cfg->mask & DEV_PWM_MASK_DUTY)
        {
          pv->duty[channel].num   = cfg->duty.num;
          pv->duty[channel].denom = cfg->duty.denom;

          err = stm32_pwm_duty(dev, channel);
          if (err)
            goto cfg_end;
        }

      if(cfg->mask & DEV_PWM_MASK_POL)
        stm32_pwm_polarity(dev, channel, cfg->pol);

      if (!(pv->config & (1 << channel)))
        {
          pv->config |= 1 << channel;
          DEVICE_REG_FIELD_IDX_SET_DEV(TIMER, pv->addr, CCER, CCE, channel);
        }
    }

  /* start counter. */
  if (start)
    {
      DEVICE_REG_UPDATE_DEV(TIMER, pv->addr, CNT, 0);
      DEVICE_REG_FIELD_SET_DEV(TIMER, pv->addr, CR1, CEN);
      DEVICE_REG_FIELD_SET_DEV(TIMER, pv->addr, EGR, UG);
    }

cfg_end:
  LOCK_RELEASE_IRQ(&dev->lock);

  rq->error = err;
  kroutine_exec(&rq->base.kr, 0);
}

static
error_t stm32_pwm_start_stop(struct device_s *dev,
                             uint_fast8_t    channel,
                             bool_t          start)
{
  struct stm32_pwm_private_s *pv = dev->drv_pv;

  if (channel > pv->chan_count)
    return -EINVAL;

  LOCK_SPIN_IRQ(&dev->lock);

  if (start)
    {
      ++pv->count[channel];
      pv->start |= 1 << channel;
    }
  else 
    {
      assert(pv->count[channel] > 0);
      --pv->count[channel];
      if(pv->count[channel] == 0)
        {
          DEVICE_REG_FIELD_IDX_CLR_DEV(TIMER, pv->addr, CCER, CCE, channel);
          pv->start &= ~(1 << channel);
        }

      if (pv->start == 0)
        DEVICE_REG_FIELD_CLR_DEV(TIMER, pv->addr, CR1, CEN);
    }

  LOCK_RELEASE_IRQ(&dev->lock);
  return 0;
}

static const struct driver_pwm_s stm32_pwm_pwm_drv =
{
  .class_   = DRIVER_CLASS_PWM,
  .f_config = stm32_pwm_config,
};

static inline
void stm32_pwm_clock_init(struct stm32_pwm_private_s *pv)
{
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

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  pv = mem_alloc(sizeof(struct stm32_pwm_private_s), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
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
      pv->chan_count = 4;
      break;

    case STM32_TIM9_ADDR:
      pv->chan_count = 2;
      break;

    case STM32_TIM10_ADDR:
    case STM32_TIM11_ADDR:
      pv->chan_count = 1;
      break;
    }

  /* FIXME: Setup clock gating. */
  stm32_pwm_clock_init(pv);

  /* FIXME: core freq. */
  if (device_get_res_freq(dev, &pv->core_freq, 0))
    goto err_mem;

  /* Stop PWM/timer */
  DEVICE_REG_FIELD_CLR_DEV(TIMER, pv->addr, CR1, CEN);

  /* Configure GPIO. */
  iomux_demux_t loc[STM32_PWM_CHANNEL_MAX];
  if (device_iomux_setup(dev, ">oc1? >oc2? >oc3? >oc4?", loc, NULL, NULL))
    goto err_mem;

  /* Check timer width. */
  DEVICE_REG_UPDATE_DEV(TIMER, pv->addr, CNT, 0xffffffff);

  if (DEVICE_REG_VALUE_DEV(TIMER, pv->addr, CNT) & 0xffff0000)
    pv->hw_width = 32;
  else
    pv->hw_width = 16;

  /* Disable interrupts. */
  DEVICE_REG_UPDATE_DEV(TIMER, pv->addr, DIER, 0);

  if (loc[0] != IOMUX_INVALID_DEMUX && pv->chan_count > 0)
    {
      /* Set capture/icompare channel PWM mode. */
      DEVICE_REG_FIELD_UPDATE_DEV(TIMER, pv->addr, CCMR1OC, OC1M, PWM_MODE_1);
      /* Set capture/compare channel as output. */
      DEVICE_REG_FIELD_UPDATE_DEV(TIMER, pv->addr, CCMR1OC, CC1S, OUTPUT);
      /* Disable CCR preload, updating CCR immediately applies. */
      DEVICE_REG_FIELD_CLR_DEV(TIMER, pv->addr, CCMR1OC, OC1PE);
    }

  /* Set up other channels. */
  if (loc[1] != IOMUX_INVALID_DEMUX && pv->chan_count > 1)
    {
      DEVICE_REG_FIELD_UPDATE_DEV(TIMER, pv->addr, CCMR1OC, OC2M, PWM_MODE_1);
      DEVICE_REG_FIELD_UPDATE_DEV(TIMER, pv->addr, CCMR1OC, CC2S, OUTPUT);
      DEVICE_REG_FIELD_CLR_DEV(TIMER, pv->addr, CCMR1OC, OC2PE);
    }
  if (loc[2] != IOMUX_INVALID_DEMUX && pv->chan_count > 2)
    {
      DEVICE_REG_FIELD_UPDATE_DEV(TIMER, pv->addr, CCMR2OC, OC3M, PWM_MODE_1);
      DEVICE_REG_FIELD_UPDATE_DEV(TIMER, pv->addr, CCMR2OC, CC3S, OUTPUT);
      DEVICE_REG_FIELD_CLR_DEV(TIMER, pv->addr, CCMR2OC, OC3PE);
    }

  if (loc[3] != IOMUX_INVALID_DEMUX && pv->chan_count > 3)
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

  dev->drv_pv = pv;
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
  mem_free(pv);
}

static
DEV_USE(stm32_pwm_use)
{
  struct device_s *dev = accessor->dev;

  switch (op)
    {
    default:
      return 0;

    case DEV_USE_START:
      return stm32_pwm_start_stop(dev, accessor->number, 1);

    case DEV_USE_STOP:
      return stm32_pwm_start_stop(dev, accessor->number, 0);

    case DEV_USE_PUT_ACCESSOR:
      if (accessor->number >= STM32_PWM_CHANNEL_MAX)
        return -EINVAL;
      return 0;
    }
}

