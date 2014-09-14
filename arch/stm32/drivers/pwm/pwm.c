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
#include <hexo/iospace.h>
#include <hexo/endian.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/class/pwm.h>
#include <device/class/timer.h>
#include <device/class/iomux.h>

#include <mutek/mem_alloc.h>
#include <mutek/kroutine.h>

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

  /* PWM common frequency. */
  struct dev_pwm_fract_s    freq;

  /* PWM channel actual count. */
  uint_fast8_t              count;

  /* PWM/timer width. */
  uint_fast8_t              hw_width;

  /* PWM duty cycle per channel. */
  struct dev_pwm_fract_s    duty[STM32_PWM_CHANNEL_MAX];
  uint_fast8_t              mask;

  /* PWM period after prescaling. */
  uint32_t                  period;
};

static
DEVPWM_FREQ(stm32_pwm_freq)
{
  struct device_s            *dev = pdev->dev;
  struct stm32_pwm_private_s *pv  = dev->drv_pv;

  if (!freq)
    return -ENOMEM;

  if (freq->num == 0)
    {
      *freq = pv->freq;
      return 0;
    }

  /* Get input frequency. */
  struct dev_freq_s freq_res;
  if (device_get_res_freq(dev, &freq_res, 0))
    return -ENOTSUP;

  /* Compute scale factor to the requested frequency. */
  uint64_t denom = freq_res.denom * freq->num;
  uint64_t scale = (freq_res.num * freq->denom + (denom >> 2)) / denom;

  /* Check if prescaler can handle the high part of the scale factor. */
  if ((scale >> pv->hw_width) & 0xffff)
    return -ENOTSUP;

  uint32_t presc, period;

  /* Compute prescaler and period of the timer. */
  if (scale < (1ULL << pv->hw_width))
    presc  = 0;
  else
    presc  = scale >> pv->hw_width;

  period = scale & ((1ULL << pv->hw_width) - 1);

  /* Save configuration. */
  pv->freq   = *freq;
  pv->period = period + 1;

  /* Configure the prescaler. */
  DEVICE_REG_UPDATE_DEV(TIMER, pv->addr, PSC, presc);

  /* Configure the period. */
  DEVICE_REG_UPDATE_DEV(TIMER, pv->addr, ARR, period);

  return 0;
}

static
DEVPWM_DUTY(stm32_pwm_duty)
{
  struct device_s            *dev = pdev->dev;
  struct stm32_pwm_private_s *pv  = dev->drv_pv;

  if (!duty)
    return -ENOMEM;

  if (channel > pv->count)
    return -EIO;

  if (duty->num > duty->denom)
    return -ENOTSUP;

  if (duty->num == 0)
    {
      *duty = pv->duty[channel];
      return 0;
    }

  pv->duty[channel] = *duty;

  uint32_t d = pv->duty[channel].num * pv->period / pv->duty[channel].denom;
  DEVICE_REG_IDX_UPDATE_DEV(TIMER, pv->addr, CCR, channel, d);

  return 0;
}

static
DEVPWM_POLARITY(stm32_pwm_polarity)
{
  struct device_s            *dev = pdev->dev;
  struct stm32_pwm_private_s *pv  = dev->drv_pv;

  if (!pol)
    return -ENOMEM;

  if (*pol == DEV_PWM_POL_NONE)
    {
      uint32_t x =
        DEVICE_REG_FIELD_IDX_VALUE_DEV(TIMER, pv->addr, CCER, CCP, channel);
      if (x & STM32_TIMER_CCER_CCP_LOW)
        *pol = DEV_PWM_POL_LOW;
      else
        *pol = DEV_PWM_POL_HIGH;
      return 0;
    }

  if ( *pol == DEV_PWM_POL_HIGH )
    DEVICE_REG_FIELD_IDX_UPDATE_DEV(TIMER, pv->addr, CCER, CCP, channel, HIGH);
  else
    DEVICE_REG_FIELD_IDX_UPDATE_DEV(TIMER, pv->addr, CCER, CCP, channel, LOW);

  return 0;
}

static
DEVPWM_START_STOP(stm32_pwm_start_stop)
{
  struct device_s            *dev = pdev->dev;
  struct stm32_pwm_private_s *pv  = dev->drv_pv;

  if (channel > pv->count || pv->freq.num == 0 || pv->duty[channel].num == 0)
    return -EIO;

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

  return 0;
}

static const struct driver_pwm_s stm32_pwm_pwm_drv =
{
  .class_       = DRIVER_CLASS_PWM,
  .f_freq       = stm32_pwm_freq,
  .f_duty       = stm32_pwm_duty,
  .f_polarity   = stm32_pwm_polarity,
  .f_start_stop = stm32_pwm_start_stop,
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

const struct driver_s stm32_pwm_drv =
{
  .desc      = "STM32 PWM",
  .f_init    = stm32_pwm_init,
  .f_cleanup = stm32_pwm_cleanup,
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

  /* Check timer width. */
  DEVICE_REG_UPDATE_DEV(TIMER, pv->addr, CNT, 0xffffffff);

  if (DEVICE_REG_VALUE_DEV(TIMER, pv->addr, CNT) & 0xffff0000)
    pv->hw_width = 32;
  else
    pv->hw_width = 16;

  /* Disable interrupts. */
  DEVICE_REG_UPDATE_DEV(TIMER, pv->addr, DIER, 0);

  /* Set auto-reload preload value. */
  DEVICE_REG_FIELD_SET_DEV(TIMER, pv->addr, CR1, ARPE);

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

  /* Master mode and internal clock selection. */
  DEVICE_REG_UPDATE_DEV(TIMER, pv->addr, SMCR, 0);

  /* Set repeat counter to 0, then generate update at each overflow. */
  DEVICE_REG_FIELD_UPDATE_DEV(TIMER, pv->addr, RCR, REP, 0);

#if 0
  uint16_t presc = ( stm32f4xx_clock_freq_ahb1 / 10000 );
  DEVICE_REG_UPDATE_DEV(TIMER, pv->addr, PSC, presc);

  uint16_t period = ( 10000 / 480 ) - 1;
  DEVICE_REG_UPDATE_DEV(TIMER, pv->addr, ARR, period);

  uint16_t duty = (period + 1) / 2;
  DEVICE_REG_IDX_UPDATE_DEV(TIMER, pv->addr, CCR, 0 /* channel */, duty);

#if 0
  DEVICE_REG_FIELD_IDX_SET_DEV(TIMER, pv->addr, CCER, CCE, 0 /* channel */);
  DEVICE_REG_FIELD_SET_DEV(TIMER, pv->addr, CR1, CEN);
#else
  pv->freq.num = 1;
  pv->duty[0].num = 1;
#endif
#endif

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

