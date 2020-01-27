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

    Copyright (c) 2014 Nicolas Pouillon <nipo@ssji.net>
*/

#define LOGK_MODULE_ID "ngpo"

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <hexo/bit.h>

#include <mutek/mem_alloc.h>
#include <arch/nrf5x/gpio.h>
#include <arch/nrf5x/gpiote.h>
#include <arch/nrf5x/timer.h>
#include <arch/nrf5x/ppi.h>

#include <device/resources.h>
#include <device/device.h>
#include <device/irq.h>
#include <device/class/gpio.h>
#include <device/class/icu.h>
#include <device/class/iomux.h>

#include <mutek/printk.h>
#include <string.h>

#define GPIO_ADDR NRF5X_GPIO_ADDR

struct nrf5x_gpio_private_s
{
#if CONFIG_DRIVER_NRF5X_GPIO_ICU_CHANNEL_COUNT || defined(CONFIG_DRIVER_NRF5X_GPIO_UNTIL)
#define GPIOTE_ADDR NRF_PERIPHERAL_ADDR(NRF5X_GPIOTE)

  struct dev_irq_src_s irq_in[1];

#if CONFIG_DRIVER_NRF5X_GPIO_ICU_CHANNEL_COUNT
  struct dev_irq_sink_s irq_out[CONFIG_DRIVER_NRF5X_GPIO_ICU_CHANNEL_COUNT];
  int8_t gpiote_pin[CONFIG_DRIVER_NRF5X_GPIO_ICU_CHANNEL_COUNT];
#endif

# if defined(CONFIG_DRIVER_NRF5X_GPIO_UNTIL)
  uint32_t last_state;
  uint32_t mask;

  dev_request_queue_root_t queue;

  struct kroutine_s until_checker;
# endif
#endif
};

DRIVER_PV(struct nrf5x_gpio_private_s);

static
int nrf5x_gpio_mode(
    enum dev_pin_driving_e mode,
    uint32_t *nrf_mode)
{
    uint32_t ret;

    switch (mode)
    {
    case DEV_PIN_DISABLED:
        ret = NRF_GPIO_PIN_CNF_DIR_INPUT
            | NRF_GPIO_PIN_CNF_INPUT_DISCONNECT;
        break;
    case DEV_PIN_PUSHPULL:
        ret = NRF_GPIO_PIN_CNF_DIR_OUTPUT
            | NRF_GPIO_PIN_CNF_DRIVE_S0S1;
        break;
    case DEV_PIN_INPUT:
        ret = NRF_GPIO_PIN_CNF_DIR_INPUT;
        break;
    case DEV_PIN_INPUT_PULLUP:
        ret = NRF_GPIO_PIN_CNF_DIR_INPUT
            | NRF_GPIO_PIN_CNF_PULL_UP;
        break;
    case DEV_PIN_INPUT_PULLDOWN:
        ret = NRF_GPIO_PIN_CNF_DIR_INPUT
            | NRF_GPIO_PIN_CNF_PULL_DOWN;
        break;
    case DEV_PIN_OPENDRAIN:
        ret = NRF_GPIO_PIN_CNF_DIR_OUTPUT
            | NRF_GPIO_PIN_CNF_DRIVE_S0D1;
        break;
    case DEV_PIN_OPENSOURCE:
        ret = NRF_GPIO_PIN_CNF_DIR_OUTPUT
            | NRF_GPIO_PIN_CNF_DRIVE_D0S1;
        break;
    case DEV_PIN_OPENDRAIN_PULLUP:
        ret = NRF_GPIO_PIN_CNF_DIR_OUTPUT
            | NRF_GPIO_PIN_CNF_DRIVE_S0D1
            | NRF_GPIO_PIN_CNF_PULL_UP;
        break;
    case DEV_PIN_OPENSOURCE_PULLDOWN:
        ret = NRF_GPIO_PIN_CNF_DIR_OUTPUT
            | NRF_GPIO_PIN_CNF_DRIVE_D0S1
            | NRF_GPIO_PIN_CNF_PULL_DOWN;
        break;
    default:
        return -ENOTSUP;
    }

    *nrf_mode = ret;
    return 0;
}

#if defined(CONFIG_DRIVER_NRF5X_GPIO_UNTIL)
static void nrf5x_gpio_mask_update(struct device_s *dev, uint32_t ref, uint32_t mask)
{
  struct nrf5x_gpio_private_s *pv = dev->drv_pv;

  logk_trace("%s %x %x %x\n", __FUNCTION__, pv->mask, mask, ref);

  uint32_t to_update = mask | pv->mask;

  while (to_update) {
    uint8_t pin = bit_ctz(to_update);
    uint32_t cnf;

    cnf = nrf_reg_get(GPIO_ADDR, NRF_GPIO_PIN_CNF(pin));
    cnf &= ~NRF_GPIO_PIN_CNF_SENSE_MASK;

    if (mask & bit(pin)) {
      logk_debug("%s sensing pin %d %s\n", __FUNCTION__, pin, bit(pin) & ref ? "low" : "high");
      cnf |= (ref & bit(pin)) ? NRF_GPIO_PIN_CNF_SENSE_LOW : NRF_GPIO_PIN_CNF_SENSE_HIGH;
    }

    nrf_reg_set(GPIO_ADDR, NRF_GPIO_PIN_CNF(pin), cnf);

    to_update &= ~bit(pin);
  }

  pv->mask = mask;

  if (pv->mask) {
    logk_debug("%s port enable\n", __FUNCTION__);

    nrf_event_clear(GPIOTE_ADDR, NRF_GPIOTE_PORT);
    nrf_it_enable(GPIOTE_ADDR, NRF_GPIOTE_PORT);
  }
}

static KROUTINE_EXEC(nrf5x_gpio_until_check)
{
  struct nrf5x_gpio_private_s *pv = KROUTINE_CONTAINER(kr, *pv, until_checker);
  struct device_s *dev = pv->irq_in[0].base.dev;

  uint32_t cur = nrf_reg_get(GPIO_ADDR, NRF_GPIO_IN);
  uint32_t mask_next = 0;
  uint32_t ref_next = 0;

  logk_trace("%s -> %x\n", __FUNCTION__, cur & pv->mask);

  {
    LOCK_SPIN_IRQ_SCOPED(&dev->lock);

    GCT_FOREACH(dev_request_queue, &pv->queue, brq, {
        struct dev_gpio_rq_s *rq = dev_gpio_rq_s_cast(brq);
        uint32_t mask = endian_le32_na_load(rq->until.mask) << rq->io_first;
        uint32_t ref = endian_le32_na_load(rq->until.data) << rq->io_first;
        mask &= bit_range(rq->io_first, rq->io_last);

        if ((cur ^ ref) & mask) {
          logk_debug("%s %p done\n", __FUNCTION__, rq);
          dev_gpio_rq_remove(&pv->queue, rq);
          dev_gpio_rq_done(rq);
        } else {
          logk_debug("%s %p still waiting\n", __FUNCTION__, rq);
          mask_next |= mask;
          ref_next |= ref;
        }
      });

  }

  nrf5x_gpio_mask_update(dev, ref_next, mask_next);
}
#endif

static DEV_GPIO_SET_MODE(nrf5x_gpio_set_mode)
{
  struct device_s *dev = gpio->dev;
  uint32_t nrf_mode;

  if (io_last > CONFIG_NRF5X_GPIO_COUNT)
    return -ERANGE;

  if (nrf5x_gpio_mode(mode, &nrf_mode))
    return -ENOTSUP;

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  uint_fast8_t count = io_last - io_first + 1;
  for (uint_fast8_t i = 0; i < count; i++) {

    if (!bit_get(mask[i / 8], i % 8))
      continue;

    logk_trace("%s %d %x\n", __FUNCTION__, i + io_first, nrf_mode);

    nrf_reg_set(GPIO_ADDR, NRF_GPIO_PIN_CNF(i + io_first), nrf_mode);
  }

  return 0;
}

static DEV_GPIO_SET_OUTPUT(nrf5x_gpio_set_output)
{
  if (io_last > CONFIG_NRF5X_GPIO_COUNT ||
      io_first > io_last)
    return -ERANGE;

  uint64_t cm, sm, tmask;
  uint64_t cmp = 0;
  uint64_t smp = 0;

  uint_fast8_t shift = io_first % 32;
  int_fast8_t mlen  = io_last - io_first + 1;

  struct device_s *dev = gpio->dev;
  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

 mask:
  /* compute mask word for next clear_mask and set_mask */
  tmask = mlen > 32 ? 0xffffffff : bit_mask(0, mlen);

 loop:
  /* compute set and clear masks */
  cm = (uint32_t)((uint16_t)~endian_le32_na_load(clear_mask) & tmask);
  cmp = ((cm << shift) | cmp);
  clear_mask += 4;

  sm = (uint32_t)(endian_le32_na_load(set_mask) & tmask);
  smp = ((sm << shift) | smp);
  set_mask += 4;

  mlen -= 32;

 last:;
  /* update DOUT register */

  uintptr_t a = NRF_GPIO_BANK_OFFSET(io_first);
  uint32_t x = nrf_reg_get(GPIO_ADDR, a + NRF_GPIO_OUT);
  uint32_t y = smp ^ (x & (smp ^ ~cmp));
  nrf_reg_set(GPIO_ADDR, a + NRF_GPIO_OUT, y);

  logk_trace("%s %d-%d clr %08llx set %08llx %08x -> %08x\n",
             __FUNCTION__, io_first, io_last, cmp, smp, x, y);

#if CONFIG_NRF5X_GPIO_COUNT > 32
  cmp >>= 32;
  smp >>= 32;

  io_first = (io_first | (32 - 1)) + 1;

  if (mlen >= 32)
    goto loop;   /* mask is still 0xffffffff, no need to recompute */

  if (io_first <= io_last)
    {
      if (mlen < 0)
	goto last;   /* last remaining bits in next register, mask
                        bits already available */

      goto mask;     /* need to compute new mask for the last word */
    }
#endif

  return 0;
}

static DEV_GPIO_GET_INPUT(nrf5x_gpio_get_input)
{
  struct device_s *dev = gpio->dev;

  if (io_last > CONFIG_NRF5X_GPIO_COUNT)
    return -ERANGE;

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  uint64_t vp;
  uint_fast8_t shift = io_first % 32;
  uintptr_t a = NRF_GPIO_BANK_OFFSET(io_first);

#if CONFIG_NRF5X_GPIO_COUNT > 32
  uint_fast8_t bf = io_first / 32;
  uint_fast8_t bl = io_last / 32;
#endif

  vp = nrf_reg_get(GPIO_ADDR, a + NRF_GPIO_IN);  /* get P0 inputs */
  vp >>= shift;

#if CONFIG_NRF5X_GPIO_COUNT > 32
  while (bf++ < bl)
    {
      uint64_t v;
      a += NRF_GPIO_BANK_OFFSET(1);
      v = nrf_reg_get(GPIO_ADDR, a + NRF_GPIO_IN);  /* get P1, P2, ... inputs */
      v = (v << (32 - shift)) | vp ;
      vp = v >> 32;

      endian_le32_na_store(data, endian_le32((uint32_t)v));
      data += 4;
    }
#endif

  endian_le32_na_store(data, endian_le32((uint32_t)vp));

  return 0;
}

#if CONFIG_DRIVER_NRF5X_GPIO_ICU_CHANNEL_COUNT

static int8_t gpiote_allocate(struct nrf5x_gpio_private_s *pv,
                              uint8_t pin)
{
  for (uint8_t i = 0; i < CONFIG_DRIVER_NRF5X_GPIO_ICU_CHANNEL_COUNT; ++i)
    if (pv->gpiote_pin[i] == pin)
      return i;

  for (uint8_t i = 0; i < CONFIG_DRIVER_NRF5X_GPIO_ICU_CHANNEL_COUNT; ++i) {
    if (pv->gpiote_pin[i] == -1) {
      pv->gpiote_pin[i] = pin;
      return i;
    }
  }

  for (uint8_t i = 0; i < CONFIG_DRIVER_NRF5X_GPIO_ICU_CHANNEL_COUNT; ++i) {
    if (pv->irq_out[i].base.link_count)
      continue;

    pv->gpiote_pin[i] = pin;
    return i;
  }

  return -1;
}

static DEV_IRQ_SINK_UPDATE(nrf5x_gpio_icu_sink_update)
{
  struct device_s *dev = sink->base.dev;
  struct nrf5x_gpio_private_s *pv = dev->drv_pv;
  uint_fast8_t te = sink - pv->irq_out;
  uint8_t pin = pv->gpiote_pin[te];

  if (te >= CONFIG_DRIVER_NRF5X_GPIO_ICU_CHANNEL_COUNT)
    return;

  if (pin >= CONFIG_NRF5X_GPIO_COUNT || pin < 0)
    return;

  for (uint8_t i = 0; i < CONFIG_DRIVER_NRF5X_GPIO_ICU_CHANNEL_COUNT; ++i) {
    if (pv->gpiote_pin[i] == pin && i != te) {
      return;
    }
  }

  if (!sense) {
    nrf_reg_set(GPIO_ADDR, NRF_GPIO_PIN_CNF(pin), 0x0);
    nrf_it_disable(GPIOTE_ADDR, NRF_GPIOTE_IN(te + CONFIG_DRIVER_NRF5X_GPIO_TE_FIRST));
    nrf_reg_set(GPIOTE_ADDR, NRF_GPIOTE_CONFIG(te + CONFIG_DRIVER_NRF5X_GPIO_TE_FIRST),
                NRF_GPIOTE_CONFIG_MODE_DISABLED);
    pv->gpiote_pin[te] = -1;
    return;
  }

  bool_t up = !!(sense & (DEV_IRQ_SENSE_HIGH_LEVEL
                          | DEV_IRQ_SENSE_RISING_EDGE
                          | DEV_IRQ_SENSE_ANY_EDGE));
  bool_t down = !!(sense & (DEV_IRQ_SENSE_LOW_LEVEL
                            | DEV_IRQ_SENSE_FALLING_EDGE
                            | DEV_IRQ_SENSE_ANY_EDGE));

  uint32_t te_config = NRF_GPIOTE_CONFIG_MODE_EVENT
    | NRF_GPIOTE_CONFIG_PSEL(pin);
  uint32_t pin_config = 0
    | NRF_GPIO_PIN_CNF_DIR_INPUT;

  if (up && down) {
    te_config |= NRF_GPIOTE_CONFIG_POLARITY_TOGGLE;
    pin_config |= NRF_GPIO_PIN_CNF_PULL_UP;
  } else if (up) {
    te_config |= NRF_GPIOTE_CONFIG_POLARITY_LOTOHI;
    pin_config |= NRF_GPIO_PIN_CNF_PULL_DOWN;
  } else {
    te_config |= NRF_GPIOTE_CONFIG_POLARITY_HITOLO;
    pin_config |= NRF_GPIO_PIN_CNF_PULL_UP;
  }

  nrf_reg_set(GPIO_ADDR, NRF_GPIO_PIN_CNF(pin), pin_config);
  nrf_reg_set(GPIOTE_ADDR, NRF_GPIOTE_CONFIG(te + CONFIG_DRIVER_NRF5X_GPIO_TE_FIRST),
              te_config);
  nrf_it_enable(GPIOTE_ADDR, NRF_GPIOTE_IN(te + CONFIG_DRIVER_NRF5X_GPIO_TE_FIRST));
  nrf_event_clear(GPIOTE_ADDR, NRF_GPIOTE_IN(te + CONFIG_DRIVER_NRF5X_GPIO_TE_FIRST));

  return;
}

#endif

#if CONFIG_DRIVER_NRF5X_GPIO_ICU_CHANNEL_COUNT

static DEV_ICU_GET_SINK(nrf5x_gpio_icu_get_sink)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_gpio_private_s *pv = dev->drv_pv;

  if (id >= CONFIG_NRF5X_GPIO_COUNT)
    return NULL;

#if CONFIG_DRIVER_NRF5X_GPIO_ICU_CHANNEL_COUNT
  int8_t i = gpiote_allocate(pv, id);
  if (i >= 0)
    return pv->irq_out + i;
#endif

  return NULL;
}

#endif

#if defined(CONFIG_DRIVER_NRF5X_GPIO_UNTIL) || CONFIG_DRIVER_NRF5X_GPIO_ICU_CHANNEL_COUNT

static DEV_IRQ_SRC_PROCESS(nrf5x_gpio_process)
{
  struct nrf5x_gpio_private_s *pv = ep->base.dev->drv_pv;
  bool_t evented = 1;
  uint32_t again = 100;

  while (evented && --again) {
    evented = 0;

#if CONFIG_DRIVER_NRF5X_GPIO_ICU_CHANNEL_COUNT
    for (int8_t te = 0; te < CONFIG_DRIVER_NRF5X_GPIO_ICU_CHANNEL_COUNT; ++te) {
      struct dev_irq_sink_s *sink = pv->irq_out + te;
      bool_t do_process = 0;

      if (pv->gpiote_pin[te] < 0)
        continue;

      bool_t gpiote_triggered = nrf_event_check(GPIOTE_ADDR, NRF_GPIOTE_IN(te + CONFIG_DRIVER_NRF5X_GPIO_TE_FIRST));
      nrf_event_clear(GPIOTE_ADDR, NRF_GPIOTE_IN(te + CONFIG_DRIVER_NRF5X_GPIO_TE_FIRST));

      if (sink->sense_link & (DEV_IRQ_SENSE_FALLING_EDGE
                              | DEV_IRQ_SENSE_RISING_EDGE
                              | DEV_IRQ_SENSE_ANY_EDGE)) {
        do_process |= gpiote_triggered;
      } else {
        bool_t value = bit_get(nrf_reg_get(GPIO_ADDR, NRF_GPIO_IN), pv->gpiote_pin[te]);
        do_process |= (sink->sense_link & DEV_IRQ_SENSE_HIGH_LEVEL) && value;
        do_process |= (sink->sense_link & DEV_IRQ_SENSE_LOW_LEVEL) && !value;
      }

      if (do_process) {
        device_irq_sink_process(sink, 0);
        evented = 1;
      }
    }
#endif

#if defined(CONFIG_DRIVER_NRF5X_GPIO_UNTIL)
    if (nrf_event_check(GPIOTE_ADDR, NRF_GPIOTE_PORT)) {
      nrf_it_disable(GPIOTE_ADDR, NRF_GPIOTE_PORT);
      nrf_event_clear(GPIOTE_ADDR, NRF_GPIOTE_PORT);
      logk_trace("%s port\n", __FUNCTION__);

      kroutine_exec(&pv->until_checker);
      evented = 1;
    }
#endif
  }
}

#endif

static DEV_IOMUX_SETUP(nrf5x_gpio_iomux_setup)
{
  if (io_id >= CONFIG_NRF5X_GPIO_COUNT)
    return -ERANGE;

  uint32_t nrf_mode;
  if (nrf5x_gpio_mode(dir, &nrf_mode))
    return -ENOTSUP;

  nrf_reg_set(GPIO_ADDR, NRF_GPIO_PIN_CNF(io_id), nrf_mode);

  return 0;
}

static DEV_GPIO_REQUEST(nrf5x_gpio_request)
{
  struct device_s *dev = gpio->dev;
  __unused__ struct nrf5x_gpio_private_s *pv = dev->drv_pv;

  logk_trace("%s\n", __FUNCTION__);

  switch (rq->type) {
  case DEV_GPIO_MODE:
    rq->error = nrf5x_gpio_set_mode(gpio,
                                     rq->io_first, rq->io_last,
                                     rq->mode.mask, rq->mode.mode);
    break;

  case DEV_GPIO_SET_OUTPUT:
    rq->error = nrf5x_gpio_set_output(gpio,
                                       rq->io_first, rq->io_last,
                                       rq->output.set_mask, rq->output.clear_mask);
    break;

  case DEV_GPIO_GET_INPUT:
    rq->error = nrf5x_gpio_get_input(gpio,
                                      rq->io_first, rq->io_last,
                                      rq->input.data);
    break;

  case DEV_GPIO_UNTIL:
# if defined(CONFIG_DRIVER_NRF5X_GPIO_UNTIL)
    if (rq->io_last < __MIN(CONFIG_NRF5X_GPIO_COUNT, 32)) {
      LOCK_SPIN_IRQ_SCOPED(&dev->lock);
      dev_gpio_rq_pushback(&pv->queue, rq);
      kroutine_exec(&pv->until_checker);
      return;
    }
#endif
    rq->error = -ENOTSUP;
  }

  dev_gpio_rq_done(rq);
}

#define nrf5x_gpio_cancel (dev_gpio_cancel_t*)&dev_driver_notsup_fcn
#define nrf5x_gpio_use dev_use_generic
#define nrf5x_gpio_icu_link device_icu_dummy_link

static DEV_INIT(nrf5x_gpio_init)
{
#if CONFIG_DRIVER_NRF5X_GPIO_ICU_CHANNEL_COUNT || defined(CONFIG_DRIVER_NRF5X_GPIO_UNTIL)
  __unused__ uintptr_t addr = 0;
  struct nrf5x_gpio_private_s *pv;

  assert(!device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL) && GPIOTE_ADDR == addr);

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  nrf_it_disable_mask(GPIOTE_ADDR, -1);

  device_irq_source_init(dev, pv->irq_in, 1, &nrf5x_gpio_process);
  if (device_irq_source_link(dev, pv->irq_in, 1, -1))
    goto err_mem;

# if CONFIG_DRIVER_NRF5X_GPIO_ICU_CHANNEL_COUNT
  for (int8_t i = 0; i < CONFIG_DRIVER_NRF5X_GPIO_ICU_CHANNEL_COUNT; ++i) {
    nrf_reg_set(GPIOTE_ADDR, NRF_GPIOTE_CONFIG(i + CONFIG_DRIVER_NRF5X_GPIO_TE_FIRST),
                NRF_GPIOTE_CONFIG_MODE_DISABLED);
    pv->gpiote_pin[i] = -1;
  }

  device_irq_sink_init(dev, pv->irq_out,
                       CONFIG_DRIVER_NRF5X_GPIO_ICU_CHANNEL_COUNT,
                       &nrf5x_gpio_icu_sink_update,
                       DEV_IRQ_SENSE_HIGH_LEVEL
                       | DEV_IRQ_SENSE_LOW_LEVEL
                       | DEV_IRQ_SENSE_RISING_EDGE
                       | DEV_IRQ_SENSE_FALLING_EDGE
                       | DEV_IRQ_SENSE_ANY_EDGE);
# endif
# if defined(CONFIG_DRIVER_NRF5X_GPIO_UNTIL)
  kroutine_init_deferred(&pv->until_checker, nrf5x_gpio_until_check);
  dev_rq_queue_init(&pv->queue);
# endif
#endif


  for (uint8_t pin = 0; pin < CONFIG_NRF5X_GPIO_COUNT; ++pin)
    nrf_reg_set(GPIO_ADDR, NRF_GPIO_PIN_CNF(pin), 0
                | NRF_GPIO_PIN_CNF_DIR_INPUT
                | NRF_GPIO_PIN_CNF_INPUT_DISCONNECT);

  return 0;

 err_mem:
#if CONFIG_DRIVER_NRF5X_GPIO_ICU_CHANNEL_COUNT || defined(CONFIG_DRIVER_NRF5X_GPIO_UNTIL)
  mem_free(pv);
#endif
  return -1;
}

static DEV_CLEANUP(nrf5x_gpio_cleanup)
{
#if CONFIG_DRIVER_NRF5X_GPIO_ICU_CHANNEL_COUNT || defined(CONFIG_DRIVER_NRF5X_GPIO_UNTIL)
  struct nrf5x_gpio_private_s  *pv = dev->drv_pv;

#if CONFIG_DRIVER_NRF5X_GPIO_ICU_CHANNEL_COUNT
  for (int8_t i = 0; i < CONFIG_DRIVER_NRF5X_GPIO_ICU_CHANNEL_COUNT; ++i)
    nrf_it_disable(GPIOTE_ADDR, NRF_GPIOTE_IN(i + CONFIG_DRIVER_NRF5X_GPIO_TE_FIRST));

#endif

  device_irq_source_unlink(dev, pv->irq_in, 1);

  mem_free(pv);
#endif

  return 0;
}

DRIVER_DECLARE(nrf5x_gpio_drv, 0, "nRF5x GPIO"
#if CONFIG_DRIVER_NRF5X_GPIO_ICU_CHANNEL_COUNT
               ",ICU"
#endif
#if defined(CONFIG_DRIVER_NRF5X_GPIO_UNTIL)
               ",UNTIL"
#endif
               , nrf5x_gpio,
#if CONFIG_DRIVER_NRF5X_GPIO_ICU_CHANNEL_COUNT
               DRIVER_ICU_METHODS(nrf5x_gpio_icu),
#endif
               DRIVER_GPIO_METHODS(nrf5x_gpio),
               DRIVER_IOMUX_METHODS(nrf5x_gpio_iomux));

