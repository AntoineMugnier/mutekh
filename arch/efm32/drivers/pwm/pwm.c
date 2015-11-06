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

    Copyright (c) 2014 Sebastien CERDAN <sebcerdan@gmail.com>

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
#include <device/class/clock.h>

#include <mutek/mem_alloc.h>
#include <mutek/kroutine.h>
#include <mutek/printk.h>

#include <arch/efm32_timer.h>

#define EFM32_PWM_HW_WIDTH 16
#define EFM32_PWM_CHANNEL_MAX 3
#define EFM32_PWM_CHANNEL_MSK 0x7

struct efm32_pwm_private_s
{
  /* PWM/timer base address. */
  uintptr_t                 addr;
  /* Start count */
  uint8_t                   count[EFM32_PWM_CHANNEL_MAX];
  /* Channel started */
  uint8_t                   start;
  /* Channel configured */
  uint8_t                   config;
  /* PWM configuration. */
  struct dev_freq_s         pwm_freq;
  struct dev_freq_ratio_s   duty[EFM32_PWM_CHANNEL_MAX];
  struct dev_freq_s         core_freq;
#ifdef CONFIG_DEVICE_CLOCK
  struct dev_clock_sink_ep_s clk_ep;
#endif
};

static error_t efm32_pwm_validate_parameter(struct device_pwm_s *pdev, struct dev_pwm_rq_s *rq)
{
  struct efm32_pwm_private_s *pv  = pdev->dev->drv_pv;
  struct dev_freq_s freq = rq->cfg[0].freq;

  /* A channel is configured before being started */
  if ((rq->chan_mask << pdev->number) & !pv->start)
    return -EINVAL; 

  uint8_t freq_msk = 0;

  for (uint8_t i = 0; i < EFM32_PWM_CHANNEL_MAX; i++)
    {
      if (!(rq->chan_mask & (1 << i)))
        continue;

      const struct dev_pwm_config_s * cfg = &rq->cfg[i];

      if (rq->mask & DEV_PWM_MASK_FREQ)
        {
          freq_msk |= 1 << i;
          /* Error on shared parameter frequency */
          if ((cfg->freq.num != freq.num) || (cfg->freq.denom != freq.denom))
            return -ENOTSUP; 
        }
    }

  if (freq_msk)
    {
      /* Error on shared parameter frequency */
      if (freq_msk != rq->chan_mask)
       return -ENOTSUP;
      if ((freq_msk << pdev->number) != pv->start)
       return -ENOTSUP;
    }

  return 0;
}

static error_t efm32_pwm_freq(struct device_s *dev)
{
  struct efm32_pwm_private_s *pv = dev->drv_pv;

  /* Compute scale factor to the requested frequency. */
  uint64_t scale = (pv->core_freq.num * pv->pwm_freq.denom) / (pv->core_freq.denom * pv->pwm_freq.num);

  uint32_t msb = scale >> EFM32_PWM_HW_WIDTH;
  
  uint8_t div = msb ? sizeof(msb) * 8 - __builtin_clz(msb) : 0;

  if (div > 10)
  /* Frequency can not be achieved */  
    return -ERANGE; 

  scale = scale / (1 << div);

  /* Update prescaler */
  uint32_t x = endian_le32(cpu_mem_read_32(pv->addr + EFM32_TIMER_CTRL_ADDR));

  EFM32_TIMER_CTRL_PRESC_SETVAL(x, div);

  uint32_t top = (scale - 1) & 0xFFFF;
 
  /* Configure PWM */
  cpu_mem_write_32(pv->addr + EFM32_TIMER_CTRL_ADDR, endian_le32(x));
  cpu_mem_write_32(pv->addr + EFM32_TIMER_TOP_ADDR, endian_le32(top));

  return 0;
}

static error_t efm32_pwm_duty(struct device_s *dev, uint_fast8_t channel)
{
  struct efm32_pwm_private_s *pv = dev->drv_pv;
  
  uint16_t top = endian_le32(cpu_mem_read_32(pv->addr + EFM32_TIMER_TOP_ADDR));

  if (pv->duty[channel].num > pv->duty[channel].denom)
    return -ERANGE; 

  uint32_t x = (pv->duty[channel].num * top) / pv->duty[channel].denom;

  cpu_mem_write_32(pv->addr + EFM32_TIMER_CC_CCV_ADDR(channel), endian_le32(x));

  return 0;
}

static void efm32_pwm_polarity(struct device_s *dev, uint_fast8_t channel, enum dev_pwm_polarity_e pol)
{
  struct efm32_pwm_private_s *pv = dev->drv_pv;

  uint32_t x = endian_le32(cpu_mem_read_32(pv->addr + EFM32_TIMER_CC_CTRL_ADDR(channel)));

  if (pol == DEV_PWM_POL_HIGH)
    x &= ~EFM32_TIMER_CC_CTRL_OUTINV;
  else
    x |= EFM32_TIMER_CC_CTRL_OUTINV;

  cpu_mem_write_32(pv->addr + EFM32_TIMER_CC_CTRL_ADDR(channel), endian_le32(x));
}

static DEV_PWM_CONFIG(efm32_pwm_config)
{
  struct device_s            *dev = pdev->dev;
  struct efm32_pwm_private_s *pv  = dev->drv_pv;

  assert(rq->chan_mask);

  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  /* Test if valid parameters */
  err = efm32_pwm_validate_parameter(pdev, rq);
  if (err)
    goto cfg_end;
 
  bool_t freq_done = 0;

  bool_t start = !(pv->config & EFM32_PWM_CHANNEL_MSK);

  for (uint8_t i = 0; i < EFM32_PWM_CHANNEL_MAX - pdev->number; i++)
    {
      if (!(rq->chan_mask & (1 << i)))
        continue;

      const struct dev_pwm_config_s * cfg = &rq->cfg[i];

      if ((rq->mask & DEV_PWM_MASK_FREQ) && !freq_done) 
        {
          pv->pwm_freq.num = cfg->freq.num;
          pv->pwm_freq.denom = cfg->freq.denom;

          err = efm32_pwm_freq(dev);

          if (err)
            goto cfg_end;

          freq_done = 1;
        }

      uint8_t channel = pdev->number + i;

      if (rq->mask & DEV_PWM_MASK_DUTY)
        {
          pv->duty[channel].num = cfg->duty.num;
          pv->duty[channel].denom = cfg->duty.denom;

          err = efm32_pwm_duty(dev, channel);
          if (err)
            goto cfg_end;
        }

      if (rq->mask & DEV_PWM_MASK_POL)
        efm32_pwm_polarity(dev, channel, cfg->pol);

      if (!(pv->config & (1 << channel)))
      /* Start channel */
        {
          pv->config |= (1 << channel);
          cpu_mem_write_32(pv->addr + EFM32_TIMER_CC_CTRL_ADDR(channel), EFM32_TIMER_CC_CTRL_MODE(PWM));
        }
    }

  if (start)
    /* Start counter */
    cpu_mem_write_32(pv->addr + EFM32_TIMER_CMD_ADDR, endian_le32(EFM32_TIMER_CMD_START));


cfg_end:

  LOCK_RELEASE_IRQ(&dev->lock);

  rq->error = err;
  kroutine_exec(&rq->base.kr);
}

#ifdef CONFIG_DEVICE_CLOCK
static DEV_CLOCK_SINK_CHANGED(efm32_pwm_clk_changed)
{
  struct efm32_pwm_private_s *pv = ep->dev->drv_pv;
  pv->core_freq = *freq;

  if (!pv->config)
    return;

  efm32_pwm_freq(ep->dev);

  for (uint8_t i = 0; i < EFM32_PWM_CHANNEL_MAX; i++)
    {
      if (pv->config & (1 << i))
        efm32_pwm_duty(ep->dev, i);
    }
}
#endif

static error_t efm32_pwm_start_stop(struct device_s *dev, uint_fast8_t channel, bool_t start)
{
  struct efm32_pwm_private_s *pv = dev->drv_pv;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);
 
  if (start)
    {
      pv->count[channel]++;
      pv->start |= 1 << channel;
    }
  else 
    {
      assert(pv->count[channel]);

      pv->count[channel]--;

      if (!pv->count[channel])
        {
          /* Stop channel */
          cpu_mem_write_32(pv->addr + EFM32_TIMER_CC_CTRL_ADDR(channel), EFM32_TIMER_CC_CTRL_MODE(OFF));
          pv->start &= ~(1 << channel);
        }

      pv->config &= ~(1 << channel);

      /* FIXME : Timer clock might be disabled here to save power */
      if (!pv->start)
        /* Stop counter */
        cpu_mem_write_32(pv->addr + EFM32_TIMER_CMD_ADDR, endian_le32(EFM32_TIMER_CMD_STOP));
    }

end:
  LOCK_RELEASE_IRQ(&dev->lock);
  return err;
}

/************************************************************************/

static DEV_INIT(efm32_pwm_init);
static DEV_CLEANUP(efm32_pwm_cleanup);
static DEV_USE(efm32_pwm_use);

DRIVER_DECLARE(efm32_pwm_drv, 0, "EFM32 PWM", efm32_pwm,
               DRIVER_PWM_METHODS(efm32_pwm));

DRIVER_REGISTER(efm32_pwm_drv);

static DEV_INIT(efm32_pwm_init)
{
  struct efm32_pwm_private_s  *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  pv = mem_alloc(sizeof(struct efm32_pwm_private_s), (mem_scope_sys));

  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));
  dev->drv_pv = pv;
  
  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    return -ENOENT;

#ifdef CONFIG_DEVICE_CLOCK
  /* enable clock */
  dev_clock_sink_init(dev, &pv->clk_ep, &efm32_pwm_clk_changed);

  struct dev_clock_link_info_s ckinfo;
  if (dev_clock_sink_link(dev, &pv->clk_ep, &ckinfo, 0, 0))
    goto err_mem;

  if (!DEV_FREQ_IS_VALID(ckinfo.freq))
    goto err_mem;
  pv->core_freq = ckinfo.freq;

  if (dev_clock_sink_hold(&pv->clk_ep, 0))
    goto err_clku;
#else
  if (device_get_res_freq(dev, &pv->core_freq, 0))
    goto err_mem;
#endif

  /* Stop pwm and clear interrupt */
  cpu_mem_write_32(pv->addr + EFM32_TIMER_CMD_ADDR, endian_le32(EFM32_TIMER_CMD_STOP));
  cpu_mem_write_32(pv->addr + EFM32_TIMER_IEN_ADDR, 0);

  /* Ctrl register configuration */
  cpu_mem_write_32(pv->addr + EFM32_TIMER_CTRL_ADDR,
                   endian_le32(EFM32_TIMER_CTRL_MODE(UP) |
                               EFM32_TIMER_CTRL_CLKSEL(PRESCHFPERCLK) |
                               EFM32_TIMER_CTRL_SYNC(NONE) |
                               EFM32_TIMER_CTRL_RISEA(NONE) |
                               EFM32_TIMER_CTRL_FALLA(NONE) |
                               EFM32_TIMER_CTRL_PRESC(DIV1)));

  /* setup pinmux */
  iomux_demux_t loc[EFM32_PWM_CHANNEL_MAX];
  if (device_iomux_setup(dev, ">cc0? >cc1? >cc2?", loc, NULL, NULL))
    goto err_clku;
 
  uint32_t x = EFM32_TIMER_CC_CTRL_MODE(OFF); 
  /* Disable Compare/Capture channels */
  for (uint8_t i = 0; i < EFM32_PWM_CHANNEL_MAX; i++)
    cpu_mem_write_32(pv->addr + EFM32_TIMER_CC_CTRL_ADDR(i), endian_le32(x));

  uint32_t route = 0;
  for(uint8_t i = 0; i < EFM32_PWM_CHANNEL_MAX; i++)
    {
      if (loc[i] != IOMUX_INVALID_DEMUX)
        {
          EFM32_TIMER_ROUTE_LOCATION_SETVAL(route, loc[i]);
          route |= EFM32_TIMER_ROUTE_CCPEN(i);
        }
    }

  cpu_mem_write_32(pv->addr + EFM32_TIMER_ROUTE_ADDR, endian_le32(route));

  dev->drv = &efm32_pwm_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;

 err_clku:
#ifdef CONFIG_DEVICE_CLOCK
  dev_clock_sink_unlink(dev, &pv->clk_ep, 1);
#endif
 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(efm32_pwm_cleanup)
{
  struct efm32_pwm_private_s *pv = dev->drv_pv;

  /* Stop pwm */
  cpu_mem_write_32(pv->addr + EFM32_TIMER_CMD_ADDR, endian_le32(EFM32_TIMER_CMD_STOP));

#ifdef CONFIG_DEVICE_CLOCK
  dev_clock_sink_release(&pv->clk_ep);
  dev_clock_sink_unlink(dev, &pv->clk_ep, 1);
#endif

  mem_free(pv);

  return 0;
}

static DEV_USE(efm32_pwm_use)
{
    struct device_accessor_s *accessor = param;

    switch (op)
      {
      case DEV_USE_START:
        return efm32_pwm_start_stop(accessor->dev, accessor->number, 1);

      case DEV_USE_STOP:
        return efm32_pwm_start_stop(accessor->dev, accessor->number, 0);

      case DEV_USE_GET_ACCESSOR:
        if (accessor->number >= EFM32_PWM_CHANNEL_MAX)
          return -ENOTSUP;
      case DEV_USE_PUT_ACCESSOR:
        return 0;

      default:
        return -ENOTSUP;
      }
}

