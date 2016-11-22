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
    License along with this program.  If not, see
    <http://www.gnu.org/licenses/>.

    Copyright (c) Nicolas Pouillon <nipo@ssji.net>, 2015
*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <hexo/interrupt.h>
#include <hexo/bit.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/irq.h>
#include <device/class/iomux.h>
#include <device/class/pwm.h>

#include <arch/nrf5x/timer.h>
#include <arch/nrf5x/ppi.h>
#include <arch/nrf5x/gpiote.h>
#include <arch/nrf5x/peripheral.h>
#include <arch/nrf5x/ids.h>

#define dprintk(...) do{}while(0)
//#define dprintk printk

#define PPI(x, y) (CONFIG_DRIVER_NRF5X_GPIO_PWM_PPI_FIRST + (x) * 2 + (y))

DRIVER_PV(struct nrf5x_gpio_pwm_context_s
{
  uintptr_t timer_addr;
  struct dev_request_dlqueue_s queue;
  bool_t running : 1;
  bool_t updated : 1;
  uint8_t use_count;

  struct dev_pwm_rq_s *rq;

  struct dev_irq_src_s irq_ep[1];

  struct dev_freq_s freq;
  uint32_t period_tk;
  uint32_t prescaler_log2;

  uint32_t ppi_disable;
  uint32_t ppi_enable;

  struct {
    struct dev_freq_ratio_s duty;
    uint8_t pin;
    bool_t gpio_init_val : 1;
    bool_t running : 1;
    uint32_t toggle_tk;
    uint32_t gpiote_config;
  } channel[CONFIG_DRIVER_NRF5X_GPIO_PWM_CHANNEL_COUNT];
});

#define OVERFLOW 3
#define GPIOTE_ID(x) (CONFIG_DRIVER_NRF5X_GPIO_PWM_TE_FIRST + (x))

#define GPIOTE_ADDR NRF_PERIPHERAL_ADDR(NRF5X_GPIOTE)

static void nrf5x_gpio_pwm_update(struct nrf5x_gpio_pwm_context_s *pv, bool_t sync);

static DEV_REQUEST_DELAYED_FUNC(nrf5x_gpio_pwm_setup)
{
  struct dev_pwm_rq_s *rq = dev_pwm_rq_s_cast(rq_);
  struct device_s *dev = accessor->dev;
  struct nrf5x_gpio_pwm_context_s *pv = dev->drv_pv;
  bool_t freq_changed = 0;
  uint32_t duty_changed = 0;
  const struct dev_pwm_config_s *config = rq->cfg;

  if (!rq->chan_mask)
    return dev_request_delayed_end(&pv->queue, rq_);

  rq->error = -ENOTSUP;

  for (uint8_t i = 0; i < CONFIG_DRIVER_NRF5X_GPIO_PWM_CHANNEL_COUNT; ++i) {
    if (!((rq->chan_mask >> i) & 1))
      continue;

    const struct dev_pwm_config_s *cfg = config++;

    if (rq->mask & DEV_PWM_MASK_FREQ) {
      if (freq_changed) {
        if (memcmp(&pv->freq, &cfg->freq, sizeof(cfg->freq))) {
          dprintk("Multiple channels with different freqs\n");
          rq->error = -EINVAL;
          return dev_request_delayed_end(&pv->queue, rq_);
        }
        continue;
      }

      pv->freq = cfg->freq;

      const uint32_t wrap = (uint64_t)16000000 * (uint64_t)pv->freq.denom / pv->freq.num;

      if (wrap > (1 << (16 + 9))) {
        dprintk("Wrap above counter max for freq %d/%d: %d\n",
                pv->freq.num, pv->freq.denom, wrap);
        rq->error = -EINVAL;
        return dev_request_delayed_end(&pv->queue, rq_);
      }

      freq_changed = 1;

      pv->prescaler_log2 = 0;
      pv->period_tk = wrap;

      while (pv->period_tk > (1 << 16)
             || (!(pv->period_tk & 1) && pv->prescaler_log2 < 4)) {
        assert(pv->prescaler_log2 <= 9);
        pv->prescaler_log2++;
        pv->period_tk >>= 1;
      }

      duty_changed |= bit_mask(0, CONFIG_DRIVER_NRF5X_GPIO_PWM_CHANNEL_COUNT);

      dprintk("PWM period %d/%d: %d ticks, wraps at %d*2^-%d\n",
             (uint32_t)pv->freq.num, (uint32_t)pv->freq.denom,
             wrap, pv->period_tk, pv->prescaler_log2);
    }

    if (rq->mask & DEV_PWM_MASK_DUTY) {
      duty_changed |= bit(i);
      pv->channel[i].duty = cfg->duty;
      dprintk("PWM config: %04d/%05d\r",
              (uint32_t)pv->channel[i].duty.num, (uint32_t)pv->channel[i].duty.denom);
    }

    if (rq->mask & DEV_PWM_MASK_POL) {
      duty_changed |= bit(i);
      pv->channel[i].gpio_init_val = cfg->pol == DEV_PWM_POL_HIGH ? 1 : 0;
    }
  }

  pv->ppi_disable = pv->ppi_enable = 0;

  for (uint8_t i = 0; i < CONFIG_DRIVER_NRF5X_GPIO_PWM_CHANNEL_COUNT; ++i) {
    if ((duty_changed >> i) & 1)
      pv->channel[i].toggle_tk = pv->period_tk * pv->channel[i].duty.num / pv->channel[i].duty.denom;

    uint32_t ppis = bit(PPI(i, 1)) | bit(PPI(i, 0));
    bool_t initval = pv->channel[i].gpio_init_val;

    if (pv->channel[i].toggle_tk == 0) {
      // Dont actually PWM, start up with value after toggle
      initval ^= 1;
      pv->ppi_disable |= ppis;
    } else if (pv->channel[i].toggle_tk >= pv->period_tk) {
      // Dont actually PWM, start up with value before toggle
      pv->ppi_disable |= ppis;
    } else {
      // Do actual PWM
      pv->ppi_enable |= ppis;
    }

    pv->channel[i].gpiote_config = NRF_GPIOTE_CONFIG_MODE_TASK
      | NRF_GPIOTE_CONFIG_PSEL(pv->channel[i].pin)
      | (initval ? NRF_GPIOTE_CONFIG_OUTINIT_HIGH : NRF_GPIOTE_CONFIG_OUTINIT_LOW)
      | (initval ? NRF_GPIOTE_CONFIG_POLARITY_LOTOHI : NRF_GPIOTE_CONFIG_POLARITY_HITOLO);

    dprintk("PWM config %d: %08x changes at %04d (%05d*%04d/%05d)\n",
           i, config,
           pv->channel[i].toggle_tk,
           pv->period_tk, pv->channel[i].duty.num, pv->channel[i].duty.denom);
  }

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  assert(!pv->rq);
  pv->rq = rq;
  pv->updated = 0;

  if (pv->running) {
    nrf_event_clear(pv->timer_addr, NRF_TIMER_COMPARE(OVERFLOW));
    nrf_short_set(pv->timer_addr, bit(NRF_TIMER_COMPARE_STOP(OVERFLOW)));
    // Stop may happen here, see below.
    nrf_it_enable(pv->timer_addr, NRF_TIMER_COMPARE(OVERFLOW));

    if (nrf_event_check(pv->timer_addr, NRF_TIMER_COMPARE(OVERFLOW)) && !pv->updated) {
      nrf_it_disable(pv->timer_addr, NRF_TIMER_COMPARE(OVERFLOW));

      // We got a race: stop happened where marked above.
      nrf5x_gpio_pwm_update(pv, 1);
    }
  } else {
    nrf5x_gpio_pwm_update(pv, 1);
  }
  CPU_INTERRUPT_RESTORESTATE;
}

static void nrf5x_gpio_pwm_update(struct nrf5x_gpio_pwm_context_s *pv, bool_t sync)
{
  struct dev_pwm_rq_s *rq = pv->rq;

  if (!rq)
    return;

  if (!nrf_it_is_enabled(pv->timer_addr, NRF_TIMER_COMPARE(OVERFLOW)) && !sync)
    return;

  pv->rq = NULL;

  pv->updated = 1;

  nrf_event_clear(pv->timer_addr, NRF_TIMER_COMPARE(OVERFLOW));
  nrf_it_disable(pv->timer_addr, NRF_TIMER_COMPARE(OVERFLOW));
  nrf_short_set(pv->timer_addr, bit(NRF_TIMER_COMPARE_CLEAR(OVERFLOW)));
  nrf_task_trigger(pv->timer_addr, NRF_TIMER_CLEAR);

  for (uint8_t i = 0; i < CONFIG_DRIVER_NRF5X_GPIO_PWM_CHANNEL_COUNT; ++i) {
    nrf_reg_set(pv->timer_addr, NRF_TIMER_CC(i), pv->channel[i].toggle_tk);

    /* nrf_reg_set(GPIOTE_ADDR, NRF_GPIOTE_CONFIG(GPIOTE_ID(i)), */
    /*             pv->channel[i].gpiote_config); */
    /* asm volatile("nop;nop;nop"); */
    /* nrf_task_trigger(GPIOTE_ADDR, NRF_GPIOTE_OUT(GPIOTE_ID(i))); */
    /* asm volatile("nop;nop;nop"); */

    nrf_reg_set(GPIOTE_ADDR, NRF_GPIOTE_CONFIG(GPIOTE_ID(i)),
                (pv->channel[i].gpiote_config & ~NRF_GPIOTE_CONFIG_POLARITY_MASK)
                | NRF_GPIOTE_CONFIG_POLARITY_TOGGLE);
  }

  nrf_reg_set(pv->timer_addr, NRF_TIMER_PRESCALER, pv->prescaler_log2);
  nrf_reg_set(pv->timer_addr, NRF_TIMER_CC(OVERFLOW), pv->period_tk);

  if (pv->ppi_enable) {
#if defined(CONFIG_ARCH_NRF51)
    // PAN-73
    nrf_reg_set(pv->timer_addr, NRF_TIMER_PPI_GPIOTE, 1);
#endif
    pv->running = 1;
    nrf_task_trigger(pv->timer_addr, NRF_TIMER_START);
  }

  //  printk("PPI: %04x %04x\n", pv->ppi_disable, pv->ppi_enable);

  nrf_ppi_disable_enable_mask(pv->ppi_disable, pv->ppi_enable);

  if (!pv->ppi_enable) {
    pv->running = 0;
    nrf_task_trigger(pv->timer_addr, NRF_TIMER_SHUTDOWN);
#if defined(CONFIG_ARCH_NRF51)
    // PAN-73
    nrf_reg_set(pv->timer_addr, NRF_TIMER_PPI_GPIOTE, 0);
#endif
  }

  rq->error = 0;
  dev_request_delayed_end(&pv->queue, &rq->base);
}

static DEV_IRQ_SRC_PROCESS(nrf5x_gpio_pwm_irq)
{
  struct device_s *dev = ep->base.dev;
  struct nrf5x_gpio_pwm_context_s *pv = dev->drv_pv;

  nrf5x_gpio_pwm_update(pv, 0);
}

static DEV_PWM_CONFIG(nrf5x_gpio_pwm_config)
{
  struct device_s *dev = pdev->dev;
  struct nrf5x_gpio_pwm_context_s *pv = dev->drv_pv;

  rq->chan_mask <<= pdev->number;

  dev_request_delayed_push(device_pwm_s_base(pdev),
                           &pv->queue, dev_pwm_rq_s_base(rq), 1);
}

/*
  TODO: add delayed device stop and low power
  add PAN workaround

#if defined(CONFIG_ARCH_NRF52)
  // PAN 78
  nrf_task_trigger(pv->addr, NRF_TIMER_SHUTDOWN);
#endif
*/

static DEV_USE(nrf5x_gpio_pwm_use)
{
  struct device_accessor_s *accessor = param;

  switch (op) {
  case DEV_USE_GET_ACCESSOR:
    if (accessor->number >= CONFIG_DRIVER_NRF5X_GPIO_PWM_CHANNEL_COUNT)
      return -ENOTSUP;
    return 0;

  case DEV_USE_LAST_NUMBER:
    accessor->number = CONFIG_DRIVER_NRF5X_GPIO_PWM_CHANNEL_COUNT - 1;
    return 0;

  default:
    return dev_use_generic(param, op);
  }
}

static DEV_INIT(nrf5x_gpio_pwm_init)
{
  struct nrf5x_gpio_pwm_context_s *pv;
  iomux_io_id_t id[CONFIG_DRIVER_NRF5X_GPIO_PWM_CHANNEL_COUNT];
  error_t err = 0;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  dev->drv_pv = pv;
  memset(pv, 0, sizeof(*pv));

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->timer_addr, NULL))
    goto free_pv;

#if defined(CONFIG_ARCH_NRF51)
  assert(pv->timer_addr != NRF_PERIPHERAL_ADDR(NRF5X_TIMER0)
         && "Timer0 not supported for PWM because of PAN-32");
#endif

  err = device_iomux_setup(dev, "p0"
#if CONFIG_DRIVER_NRF5X_GPIO_PWM_CHANNEL_COUNT > 1
                           " p1"
# if CONFIG_DRIVER_NRF5X_GPIO_PWM_CHANNEL_COUNT > 2
                           " p2"
# endif
#endif
                           , NULL, id, NULL);
  if (err)
    goto free_pv;

  device_irq_source_init(dev, pv->irq_ep, 1, &nrf5x_gpio_pwm_irq);

  if (device_irq_source_link(dev, pv->irq_ep, 1, -1))
    goto free_pv;

  pv->running = 0;

  nrf_reg_set(pv->timer_addr, NRF_TIMER_BITMODE, NRF_TIMER_BITMODE_16);

  nrf_it_disable_mask(pv->timer_addr, -1);
  dev_request_delayed_init(&pv->queue, nrf5x_gpio_pwm_setup);

  nrf_reg_set(pv->timer_addr, NRF_TIMER_MODE, NRF_TIMER_MODE_TIMER);
  nrf_reg_set(pv->timer_addr, NRF_TIMER_CC(OVERFLOW), 0);

  for (uint8_t i = 0; i < CONFIG_DRIVER_NRF5X_GPIO_PWM_CHANNEL_COUNT; ++i) {
    if (id[i] == IOMUX_INVALID_ID)
      continue;

    pv->channel[i].pin = id[i];

    nrf_reg_set(pv->timer_addr, NRF_TIMER_CC(i), 0);

    nrf_ppi_setup(PPI(i, 1),
                    pv->timer_addr, NRF_TIMER_COMPARE(i),
                    GPIOTE_ADDR, NRF_GPIOTE_OUT(GPIOTE_ID(i)));

    nrf_ppi_setup(PPI(i, 0),
                    pv->timer_addr, NRF_TIMER_COMPARE(OVERFLOW),
                    GPIOTE_ADDR, NRF_GPIOTE_OUT(GPIOTE_ID(i)));
  }

  return 0;

 free_pv:
  mem_free(pv);
  return err;
}

static DEV_CLEANUP(nrf5x_gpio_pwm_cleanup)
{
  struct nrf5x_gpio_pwm_context_s *pv = dev->drv_pv;

  if (!dev_request_delayed_isidle(&pv->queue))
    return -EBUSY;

  dev_request_delayed_cleanup(&pv->queue);

  nrf_it_disable_mask(pv->timer_addr, -1);
  nrf_task_trigger(pv->timer_addr, NRF_TIMER_SHUTDOWN);

  for (uint8_t i = 0; i < CONFIG_DRIVER_NRF5X_GPIO_PWM_CHANNEL_COUNT; ++i) {
    nrf_reg_set(GPIOTE_ADDR, NRF_GPIOTE_CONFIG(GPIOTE_ID(i)), 0);
    nrf_ppi_disable_mask(bit(PPI(i, 0)) | bit(PPI(i, 1)));
  }

  device_irq_source_unlink(dev, pv->irq_ep, 1);

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(nrf5x_gpio_pwm_drv, 0, "nRF5x GPIO PWM", nrf5x_gpio_pwm,
               DRIVER_PWM_METHODS(nrf5x_gpio_pwm));

DRIVER_REGISTER(nrf5x_gpio_pwm_drv);

