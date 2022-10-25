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

    Copyright (c) 2016 Julien Peeters <contact@julienpeeters.net>
*/

#define LOGK_MODULE_ID "eadc"

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <hexo/interrupt.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/device.h>
#include <device/driver.h>
#include <device/resources.h>
#include <device/irq.h>
#include <device/clock.h>

#include <device/class/valio.h>
#include <device/valio/adc.h>

#include <device/valio/adc.h>

#include <arch/efm32/adc.h>

/*
//  Typical instance:

#ifdef CONFIG_DRIVER_EFM32_ADC

DEV_DECLARE_STATIC(
    adc0_dev, "adc0", 0, efm32_adc_drv,
    DEV_STATIC_RES_MEM(EFM32_ADC0_ADDR, EFM32_ADC0_ADDR + 0x400),
# ifdef CONFIG_DEVICE_CLOCK
    DEV_STATIC_RES_CLK_SRC("/recmu", EFM32_CLOCK_ADC0, 0),
# else
    DEV_STATIC_RES_FREQ(14000000, 1),
# endif

    DEV_STATIC_RES_DEV_ICU("/cpu"),
    DEV_STATIC_RES_IRQ(0, EFM32_IRQ_ADC0, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),
    DEV_STATIC_RES_UINT_ARRAY_PARAM("config",
        // External Channel 0
        EFM32_ADC_SINGLECTRL_RES_SHIFT_VAL(12BITS)
        | EFM32_ADC_SINGLECTRL_INPUTSEL_SHIFT_VAL(CH0_CH0CH1)
        | EFM32_ADC_SINGLECTRL_REF_SHIFT_VAL(2V5)
        | EFM32_ADC_SINGLECTRL_AT_SHIFT_VAL(64CYCLES),
        // External Channel 7
        EFM32_ADC_SINGLECTRL_RES_SHIFT_VAL(12BITS)
        | EFM32_ADC_SINGLECTRL_INPUTSEL_SHIFT_VAL(CH7)
        | EFM32_ADC_SINGLECTRL_REF_SHIFT_VAL(2V5)
        | EFM32_ADC_SINGLECTRL_AT_SHIFT_VAL(64CYCLES),
        // Internal temperature
        EFM32_ADC_SINGLECTRL_RES_SHIFT_VAL(12BITS)
        | EFM32_ADC_SINGLECTRL_INPUTSEL_SHIFT_VAL(TEMP)
        | EFM32_ADC_SINGLECTRL_REF_SHIFT_VAL(1V25)
        | EFM32_ADC_SINGLECTRL_AT_SHIFT_VAL(256CYCLES),
        // VDD
        EFM32_ADC_SINGLECTRL_RES_SHIFT_VAL(12BITS)
        | EFM32_ADC_SINGLECTRL_INPUTSEL_SHIFT_VAL(VDDDIV3)
        | EFM32_ADC_SINGLECTRL_REF_SHIFT_VAL(1V25)
        | EFM32_ADC_SINGLECTRL_AT_SHIFT_VAL(256CYCLES),
        ),
    );

// Call valio driver with an array of 4 values and mask = 0xf.

#endif

*/

struct efm32_adc_private_s
{
  /* The device memory address */
  uintptr_t                  addr;

  /* Clock & power end-point */
  struct dev_clock_sink_ep_s clk_ep;
  struct dev_freq_s          freq;

  /* IRQ source (i.e. end of conversion) */
  struct dev_irq_src_s       src_ep[1];

  /* Request queue */
  dev_request_queue_root_t   queue;

  /* samples of current request */
  int16_t                    *samples;

  /* Configuration array of channels */
  uintptr_t const *          config;
  uint16_t                   config_count;

  /* Pending channel */
  uint16_t                   pending;
};

#if defined(CONFIG_DEVICE_CLOCK_VARFREQ)

static
void efm32_adc_clk_changed(struct efm32_adc_private_s * pv)
{
  uintptr_t a = pv->addr + EFM32_ADC_CTRL_ADDR;
  uint32_t  x = endian_le32(cpu_mem_read_32(a));
  EFM32_ADC_CTRL_TIMEBASE_SET(x, pv->freq.num / pv->freq.denom / 1000000);
  cpu_mem_write_32(a, endian_le32(x));
}

#endif

static
void efm32_adc_sample_next(struct efm32_adc_private_s * pv)
{
  uint_fast8_t index = bit_ctz(pv->pending);
  pv->pending &= ~bit(index);

  uint32_t cfg = pv->config[index];
  cfg &= ~EFM32_ADC_SINGLECTRL_ADJ;
  cfg &= ~EFM32_ADC_SINGLECTRL_REP;

  logk_debug("Channel %d config %08x", index, cfg);

  cpu_mem_write_32(pv->addr + EFM32_ADC_SINGLECTRL_ADDR, endian_le32(cfg));
  cpu_mem_write_32(pv->addr + EFM32_ADC_IFC_ADDR, -1);
  cpu_mem_write_32(pv->addr + EFM32_ADC_CMD_ADDR,
                   endian_le32(EFM32_ADC_CMD_SINGLESTART));
}

static
bool_t efm32_adc_request_next(struct efm32_adc_private_s * pv)
{
  struct dev_valio_rq_s  *   rq;
  struct valio_adc_group_s * group;

  rq = dev_valio_rq_head(&pv->queue);
  if (!rq)
    return 0;

  group = rq->data;

  pv->samples = group->value;
  pv->pending = group->mask & bit_mask(0, pv->config_count);

#if defined(CONFIG_DEVICE_CLOCK_GATING)
  dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER_CLOCK);
#endif

  cpu_mem_write_32(pv->addr + EFM32_ADC_IEN_ADDR,
                   endian_le32(EFM32_ADC_IEN_SINGLE));

  efm32_adc_sample_next(pv);
  return 1;
}

static
DEV_VALIO_REQUEST(efm32_adc_request)
{
  struct device_s *            dev = accessor->dev;
  struct efm32_adc_private_s * pv  = dev->drv_pv;

  struct valio_adc_group_s * group = rq->data;

  rq->error = 0;

  if (!group->mask)
    {
      rq->error = -ENOTSUP;
      dev_valio_rq_done(rq);
      return;
    }

  if (rq->attribute != VALIO_ADC_VALUE || rq->type != DEVICE_VALIO_READ)
    {
      rq->error = -ENOTSUP;
      dev_valio_rq_done(rq);
      return;
    }

  LOCK_SPIN_IRQ(&dev->lock);

  bool_t empty = dev_rq_queue_isempty(&pv->queue);
  dev_valio_rq_pushback(&pv->queue, rq);

  if (empty)
    efm32_adc_request_next(pv);

  LOCK_RELEASE_IRQ(&dev->lock);
}

static
DEV_VALIO_CANCEL(efm32_adc_cancel)
{
  return -ENOTSUP;
}

static
DEV_IRQ_SRC_PROCESS(efm32_adc_irq)
{
  struct device_s *            dev = ep->base.dev;
  struct efm32_adc_private_s * pv  = dev->drv_pv;

  struct dev_valio_rq_s *    rq;

  uint32_t x;

  lock_spin(&dev->lock);

  rq = dev_valio_rq_head(&pv->queue);
  if (!rq)
    goto stop;

  x = endian_le32(cpu_mem_read_32(pv->addr + EFM32_ADC_IF_ADDR));
  if (!x)
    goto end;

  cpu_mem_write_32(pv->addr + EFM32_ADC_IFC_ADDR, endian_le32(x));

  if (x & EFM32_ADC_IF_SINGLE)
    {
      uint16_t s = cpu_mem_read_32(pv->addr + EFM32_ADC_SINGLEDATA_ADDR);

      *pv->samples++ = s;

      logk_debug("Value 0x%04x", s);

      cpu_mem_write_32(pv->addr + EFM32_ADC_CMD_ADDR,
                       endian_le32(EFM32_ADC_CMD_SINGLESTOP));

      if (!pv->pending)
        goto callback;
      goto again;
    }

callback:
  dev_valio_rq_pop(&pv->queue);
  rq->error = 0;

  lock_release(&dev->lock);
  dev_valio_rq_done(rq);
  lock_spin(&dev->lock);

  if (!efm32_adc_request_next(pv))
    goto stop;
  goto end;

again:
  efm32_adc_sample_next(pv);
  goto end;

stop:
  cpu_mem_write_32(pv->addr + EFM32_ADC_IEN_ADDR, 0);
  cpu_mem_write_32(pv->addr + EFM32_ADC_CMD_ADDR,
                   endian_le32(EFM32_ADC_CMD_SINGLESTOP));

#if defined(CONFIG_DEVICE_CLOCK_GATING)
  dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER);
#endif

end:
  lock_release(&dev->lock);
}

static
DEV_INIT(efm32_adc_init)
{
  struct efm32_adc_private_s * pv;

  uintptr_t const * config;
  uint16_t          config_count;

  error_t err;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  err = device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL);
  if (err)
    goto mem_cleanup;

  err = device_get_param_uint_array(dev, "config", &config_count, &config);
  if (err)
    goto mem_cleanup;

  err = dev_drv_clock_init(dev, &pv->clk_ep, 0, DEV_CLOCK_EP_FREQ_NOTIFY |
                           DEV_CLOCK_EP_POWER_CLOCK | DEV_CLOCK_EP_GATING_SYNC,
                           &pv->freq);
  if (err)
    goto mem_cleanup;

  assert(pv->freq.denom == 1);

  device_irq_source_init(dev, pv->src_ep, 1, &efm32_adc_irq);

  err = device_irq_source_link(dev, pv->src_ep, 1, -1);
  if (err)
    goto clk_cleanup;

  uint_fast8_t i, log2_div = 0;
  for (i = 0; i < 7; ++i)
    {
      /* ADC clock @ 13MHz max! */
      if ((pv->freq.num >> log2_div) > 13000000)
        ++log2_div;
      else
        break;
    }

  if (i == 7)
    {
      err = -EINVAL;
      goto irq_cleanup;
    }

  uint_fast8_t const div      = 1 << log2_div;
  uint64_t     const timebase = ((uint64_t)pv->freq.num / pv->freq.denom / 1000000) >> log2_div;

  uint32_t x = CONFIG_DRIVER_EFM32_ADC_CTRL;
  EFM32_ADC_CTRL_TIMEBASE_SET(x, timebase);
  EFM32_ADC_CTRL_PRESC_SET(x, div - 1);
  EFM32_ADC_CTRL_OVRSEL_SET(x, X4096);
  cpu_mem_write_32(pv->addr + EFM32_ADC_CTRL_ADDR, endian_le32(x));
  cpu_mem_write_32(pv->addr + EFM32_ADC_CMD_ADDR, endian_le32(EFM32_ADC_CMD_SINGLESTOP));

  dev_rq_queue_init(&pv->queue);

  pv->config       = config;
  pv->config_count = config_count;

  dev->drv_pv = pv;

#if defined(CONFIG_DEVICE_CLOCK_GATING)
  dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER);
#endif

  return 0;

irq_cleanup:
  device_irq_source_unlink(dev, pv->src_ep, 1);
clk_cleanup:
  dev_drv_clock_cleanup(dev, &pv->clk_ep);
mem_cleanup:
  mem_free(pv);
  return err;
}

static
DEV_CLEANUP(efm32_adc_cleanup)
{
  struct efm32_adc_private_s * pv = dev->drv_pv;

  if (!dev_rq_queue_isempty(&pv->queue))
    return -EBUSY;

  dev_drv_clock_cleanup(dev, &pv->clk_ep);
  device_irq_source_unlink(dev, pv->src_ep, 1);
  mem_free(pv);

  return 0;
}

static
DEV_USE(efm32_adc_use)
{
  switch (op)
    {
    default:
      return dev_use_generic(param, op);

#if defined(CONFIG_DEVICE_CLOCK_VARFREQ)
    case DEV_USE_CLOCK_SINK_FREQ_CHANGED:
      {
        struct dev_clock_notify_s *  chg  = param;
        struct dev_clock_sink_ep_s * sink = chg->sink;
        struct device_s *            dev  = sink->dev;
        struct efm32_adc_private_s * pv   = dev->drv_pv;

        pv->freq = chg->freq;
        efm32_adc_clk_changed(pv);
        return 0;
      }
#endif
    }
}

DRIVER_PV(struct efm32_adc_private_s);

DRIVER_DECLARE(efm32_adc_drv, 0, "EFM32 adc", efm32_adc,
               DRIVER_VALIO_METHODS(efm32_adc));

DRIVER_REGISTER(efm32_adc_drv);
