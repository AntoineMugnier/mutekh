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

#include <arch/stm32/timer.h>


#define STM32_PWM_CHANNEL_MAX 4
#define STM32_PWM_CHANNEL_MASK ((1<<STM32_PWM_CHANNEL_MAX)-1)

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

  struct dev_freq_s const *freq = &rq->cfg[0].freq;

  uint8_t freq_mask = 0;
  uint8_t const max = STM32_PWM_CHANNEL_MAX - pdev->number;
  uint8_t ci;
  for (ci = 0; ci < max; ++ci)
    {
      if (!(rq->chan_mask & (1 << ci)))
        continue;

      const struct dev_pwm_config_s *cfg = &rq->cfg[ci];
      if (rq->mask & DEV_PWM_MASK_FREQ)
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
  cpu_mem_write_32( ( (((pv->addr))) + (STM32_TIMER_RCR_ADDR) ), endian_le32(0) );
  do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_TIMER_CR1_ADDR) ))); STM32_TIMER_CR1_CKD_SET( (_reg), NO_DIV ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_TIMER_CR1_ADDR) ), endian_le32(_reg) ); } while (0);
  cpu_mem_write_32( ( (((pv->addr))) + (STM32_TIMER_PSC_ADDR) ), endian_le32(presc) );
  cpu_mem_write_32( ( (((pv->addr))) + (STM32_TIMER_ARR_ADDR) ), endian_le32(period) );

  return 0;
}

static
error_t stm32_pwm_duty(struct device_s *dev, uint_fast8_t channel)
{
  struct stm32_pwm_private_s *pv = dev->drv_pv;

  if (pv->duty[channel].num > pv->duty[channel].denom)
    return -ERANGE;

  uint32_t const period = endian_le32(cpu_mem_read_32(( (((pv->addr))) + (STM32_TIMER_ARR_ADDR) )));
  uint32_t const ratio = (uint64_t)pv->duty[channel].num * period / pv->duty[channel].denom;
  cpu_mem_write_32( ( (((pv->addr))) + (STM32_TIMER_CCR_ADDR(channel)) ), endian_le32(ratio) );

  return 0;
}

static
void stm32_pwm_polarity(struct device_s         *dev,
                        uint_fast8_t            channel,
                        enum dev_pwm_polarity_e pol)
{
  struct stm32_pwm_private_s *pv = dev->drv_pv;

  if ( pol == DEV_PWM_POL_HIGH )
    do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_TIMER_CCER_ADDR) ))); STM32_TIMER_CCER_CCP_SET( channel, _reg, HIGH ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_TIMER_CCER_ADDR) ), endian_le32(_reg) ); } while (0);
  else
    do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_TIMER_CCER_ADDR) ))); STM32_TIMER_CCER_CCP_SET( channel, _reg, LOW ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_TIMER_CCER_ADDR) ), endian_le32(_reg) ); } while (0);
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

      struct dev_pwm_config_s const *cfg = &rq->cfg[ci];
      if ((rq->mask & DEV_PWM_MASK_FREQ) && !fdone)
        {
          pv->freq.num   = cfg->freq.num;
          pv->freq.denom = cfg->freq.denom;

          err = stm32_pwm_freq(dev);
          if (err)
            goto cfg_end;

          fdone = 1;
        }

      uint8_t channel = pdev->number + ci;
      if (rq->mask & DEV_PWM_MASK_DUTY)
        {
          pv->duty[channel].num   = cfg->duty.num;
          pv->duty[channel].denom = cfg->duty.denom;

          err = stm32_pwm_duty(dev, channel);
          if (err)
            goto cfg_end;
        }

      if(rq->mask & DEV_PWM_MASK_POL)
        stm32_pwm_polarity(dev, channel, cfg->pol);

      if (!(pv->config & (1 << channel)))
        {
          pv->config |= 1 << channel;
          do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_TIMER_CCER_ADDR) ))); STM32_TIMER_CCER_CCE_SET( channel, (_reg), 1 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_TIMER_CCER_ADDR) ), endian_le32(_reg) ); } while (0);
        }
    }

  /* start counter. */
  if (start)
    {
      cpu_mem_write_32( ( (((pv->addr))) + (STM32_TIMER_CNT_ADDR) ), endian_le32(0) );
      do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_TIMER_CR1_ADDR) ))); STM32_TIMER_CR1_CEN_SET( (_reg), 1 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_TIMER_CR1_ADDR) ), endian_le32(_reg) ); } while (0);
      do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_TIMER_EGR_ADDR) ))); STM32_TIMER_EGR_UG_SET( (_reg), 1 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_TIMER_EGR_ADDR) ), endian_le32(_reg) ); } while (0);
    }

cfg_end:
  LOCK_RELEASE_IRQ(&dev->lock);

  rq->error = err;
  kroutine_exec(&rq->base.kr);
}

static
error_t stm32_pwm_start_stop(struct device_s *dev,
                             uint_fast8_t    channel,
                             bool_t          start)
{
  struct stm32_pwm_private_s *pv = dev->drv_pv;

  if (start)
    {
      ++pv->count[channel];
      pv->start |= 1 << channel;
      return 0;
    }
  else if (pv->count[channel] > 0)
    {
      --pv->count[channel];
      if (pv->count[channel] == 0)
        {
          do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_TIMER_CCER_ADDR) ))); STM32_TIMER_CCER_CCE_SET( channel, (_reg), 1 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_TIMER_CCER_ADDR) ), endian_le32(_reg) ); } while (0);
          pv->start &= ~(1 << channel);
        }

      if (pv->start == 0)
        do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_TIMER_CR1_ADDR) ))); STM32_TIMER_CR1_CEN_SET( (_reg), 0 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_TIMER_CR1_ADDR) ), endian_le32(_reg) ); } while (0);
      return 0;
    }

  return -EBUSY;
}

/************************************************************************/


static DEV_INIT(stm32_pwm_init)
{
  struct stm32_pwm_private_s *pv;


  pv = mem_alloc(sizeof(struct stm32_pwm_private_s), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

  struct dev_resource_s *r = device_res_get_from_name(dev, DEV_RES_UINT_PARAM, 0, "channel-count");
  if (r)
    pv->chan_count = r->u.uint_param.value;
  else
    goto err_mem;

  /* FIXME: core freq. */
  if (device_get_res_freq(dev, &pv->core_freq, 0))
    goto err_mem;

  /* Stop PWM/timer */
  do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_TIMER_CR1_ADDR) ))); STM32_TIMER_CR1_CEN_SET( (_reg), 0 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_TIMER_CR1_ADDR) ), endian_le32(_reg) ); } while (0);

  /* Configure GPIO. */
  iomux_demux_t loc[STM32_PWM_CHANNEL_MAX];
  if (device_iomux_setup(dev, ">oc1? >oc2? >oc3? >oc4?", loc, NULL, NULL))
    goto err_mem;

  /* Check timer width. */
  cpu_mem_write_32( ( (((pv->addr))) + (STM32_TIMER_CNT_ADDR) ), endian_le32(0xffffffff) );

  if (endian_le32(cpu_mem_read_32(( (((pv->addr))) + (STM32_TIMER_CNT_ADDR) ))) & 0xffff0000)
    pv->hw_width = 32;
  else
    pv->hw_width = 16;

  /* Disable interrupts. */
  cpu_mem_write_32( ( (((pv->addr))) + (STM32_TIMER_DIER_ADDR) ), endian_le32(0) );

  if (loc[0] != IOMUX_INVALID_DEMUX && pv->chan_count > 0)
    {
      /* Set capture/icompare channel PWM mode. */
      do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_TIMER_CCMR1OC_ADDR) ))); STM32_TIMER_CCMR1OC_OC1M_SET( (_reg), PWM_MODE_1 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_TIMER_CCMR1OC_ADDR) ), endian_le32(_reg) ); } while (0);
      /* Set capture/compare channel as output. */
      do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_TIMER_CCMR1OC_ADDR) ))); STM32_TIMER_CCMR1OC_CC1S_SET( (_reg), OUTPUT ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_TIMER_CCMR1OC_ADDR) ), endian_le32(_reg) ); } while (0);
      /* Disable CCR preload, updating CCR immediately applies. */
      do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_TIMER_CCMR1OC_ADDR) ))); STM32_TIMER_CCMR1OC_OC1PE_SET( (_reg), 0 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_TIMER_CCMR1OC_ADDR) ), endian_le32(_reg) ); } while (0);
    }

  /* Set up other channels. */
  if (loc[1] != IOMUX_INVALID_DEMUX && pv->chan_count > 1)
    {
      do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_TIMER_CCMR1OC_ADDR) ))); STM32_TIMER_CCMR1OC_OC2M_SET( (_reg), PWM_MODE_1 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_TIMER_CCMR1OC_ADDR) ), endian_le32(_reg) ); } while (0);
      do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_TIMER_CCMR1OC_ADDR) ))); STM32_TIMER_CCMR1OC_CC2S_SET( (_reg), OUTPUT ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_TIMER_CCMR1OC_ADDR) ), endian_le32(_reg) ); } while (0);
      do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_TIMER_CCMR1OC_ADDR) ))); STM32_TIMER_CCMR1OC_OC2PE_SET( (_reg), 0 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_TIMER_CCMR1OC_ADDR) ), endian_le32(_reg) ); } while (0);
    }
  if (loc[2] != IOMUX_INVALID_DEMUX && pv->chan_count > 2)
    {
      do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_TIMER_CCMR2OC_ADDR) ))); STM32_TIMER_CCMR2OC_OC3M_SET( (_reg), PWM_MODE_1 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_TIMER_CCMR2OC_ADDR) ), endian_le32(_reg) ); } while (0);
      do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_TIMER_CCMR2OC_ADDR) ))); STM32_TIMER_CCMR2OC_CC3S_SET( (_reg), OUTPUT ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_TIMER_CCMR2OC_ADDR) ), endian_le32(_reg) ); } while (0);
      do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_TIMER_CCMR2OC_ADDR) ))); STM32_TIMER_CCMR2OC_OC3PE_SET( (_reg), 0 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_TIMER_CCMR2OC_ADDR) ), endian_le32(_reg) ); } while (0);
    }

  if (loc[3] != IOMUX_INVALID_DEMUX && pv->chan_count > 3)
    {
      do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_TIMER_CCMR2OC_ADDR) ))); STM32_TIMER_CCMR2OC_OC4M_SET( (_reg), PWM_MODE_1 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_TIMER_CCMR2OC_ADDR) ), endian_le32(_reg) ); } while (0);
      do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_TIMER_CCMR2OC_ADDR) ))); STM32_TIMER_CCMR2OC_CC4S_SET( (_reg), OUTPUT ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_TIMER_CCMR2OC_ADDR) ), endian_le32(_reg) ); } while (0);
      do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_TIMER_CCMR2OC_ADDR) ))); STM32_TIMER_CCMR2OC_OC4PE_SET( (_reg), 0 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_TIMER_CCMR2OC_ADDR) ), endian_le32(_reg) ); } while (0);
    }

  /* Upcounting. */
  do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_TIMER_CR1_ADDR) ))); STM32_TIMER_CR1_DIR_SET( (_reg), 0 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_TIMER_CR1_ADDR) ), endian_le32(_reg) ); } while (0);

  /* ster mode and internal clock selection. */
  cpu_mem_write_32( ( (((pv->addr))) + (STM32_TIMER_SMCR_ADDR) ), endian_le32(0) );

  /* Set repeat counter to 0, then generate update at each overflow. */
  do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_TIMER_RCR_ADDR) ))); STM32_TIMER_RCR_REP_SET( (_reg), 0 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_TIMER_RCR_ADDR) ), endian_le32(_reg) ); } while (0);

  dev->drv_pv = pv;

  return 0;

err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(stm32_pwm_cleanup)
{
  struct stm32_pwm_private_s *pv = dev->drv_pv;

  do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_TIMER_CR1_ADDR) ))); STM32_TIMER_CR1_CEN_SET( (_reg), 0 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_TIMER_CR1_ADDR) ), endian_le32(_reg) ); } while (0);
  mem_free(pv);

  return 0;
}

static DEV_USE(stm32_pwm_use)
{
  struct device_accessor_s *accessor = param;

  switch (op)
    {
    case DEV_USE_START:
      return stm32_pwm_start_stop(accessor->dev, accessor->number, 1);

    case DEV_USE_STOP:
      return stm32_pwm_start_stop(accessor->dev, accessor->number, 0);

    case DEV_USE_GET_ACCESSOR:
      if (accessor->number >= STM32_PWM_CHANNEL_MAX)
        return -ENOTSUP;
    case DEV_USE_LAST_NUMBER:
      accessor->number = STM32_PWM_CHANNEL_MAX - 1;
      return 0;
    default:
      return dev_use_generic(param, op);
    }
}

DRIVER_DECLARE(stm32_pwm_drv, 0, "STM32 PWM", stm32_pwm,
               DRIVER_PWM_METHODS(stm32_pwm));

DRIVER_REGISTER(stm32_pwm_drv);

