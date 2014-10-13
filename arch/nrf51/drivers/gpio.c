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

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>

#include <mutek/mem_alloc.h>
#include <arch/nrf51/gpio.h>
#include <arch/nrf51/gpiote.h>
#include <arch/nrf51/timer.h>
#include <arch/nrf51/ppi.h>
#include <arch/ppi.h>

#include <device/resources.h>
#include <device/device.h>
#include <device/irq.h>
#include <device/class/gpio.h>
#include <device/class/icu.h>
#include <device/class/pwm.h>
#include <device/class/iomux.h>

#include <mutek/printk.h>
#include <string.h>

#define dprintk(...) do{}while(0)
//#define dprintk printk

#define GPIO_ADDR ((uintptr_t)0x50000000)
#define GPIO_COUNT 31

#if defined(CONFIG_DRIVER_NRF51_GPIO_ICU)
#define GPIOTE_ADDR NRF_PERIPHERAL_ADDR(NRF51_GPIOTE)
#define GPIOTE_COUNT 4

#define PWM_COUNT 3
#define OVERFLOW 4

struct nrf51_gpio_private_s
{
  struct dev_irq_ep_s irq_in[1];

#if defined(CONFIG_DRIVER_NRF51_GPIO_ICU)
  struct dev_irq_ep_s irq_out[GPIOTE_COUNT];
#endif

#if defined(CONFIG_DRIVER_NRF51_GPIO_ICU) ||  defined(CONFIG_DRIVER_NRF51_GPIO_PWM)
  int8_t gpiote_pin[GPIOTE_COUNT];
#endif

# if defined(CONFIG_DRIVER_NRF51_GPIO_PWM)
  uintptr_t pwm_timer_addr;
  uint32_t pwm_div_pow2;
  uint32_t pwm_wrap_tk;

  struct {
    uint32_t toggle_tk;
    uint8_t ppi[2];
    uint8_t start_count;
    uint8_t gpio_init_val : 1;
    uint8_t enabled : 1;
    uint8_t gpiote : 6;
  } pwm[PWM_COUNT];
# endif

# if defined(CONFIG_DRIVER_NRF51_GPIO_INPUT_RANGE)
  struct dev_irq_ep_s range_irq_out;
  uint32_t range_mask;
  enum dev_irq_sense_modes_e range_mode;
# endif
};
#endif

static
int nrf51_gpio_mode(
    enum dev_pin_driving_e mode,
    uint32_t *nrf_mode)
{
    uint32_t ret;

    switch (mode)
    {
    case DEV_PIN_DISABLED:
        ret = NRF51_GPIO_PIN_CNF_DIR_INPUT
            | NRF51_GPIO_PIN_CNF_INPUT_DISCONNECT;
        break;
    case DEV_PIN_PUSHPULL:
        ret = NRF51_GPIO_PIN_CNF_DIR_OUTPUT
            | NRF51_GPIO_PIN_CNF_DRIVE_S0S1;
        break;
    case DEV_PIN_INPUT:
        ret = NRF51_GPIO_PIN_CNF_DIR_INPUT;
        break;
    case DEV_PIN_INPUT_PULLUP:
        ret = NRF51_GPIO_PIN_CNF_DIR_INPUT
            | NRF51_GPIO_PIN_CNF_PULL_UP;
        break;
    case DEV_PIN_INPUT_PULLDOWN:
        ret = NRF51_GPIO_PIN_CNF_DIR_INPUT
            | NRF51_GPIO_PIN_CNF_PULL_DOWN;
        break;
    case DEV_PIN_OPENDRAIN:
        ret = NRF51_GPIO_PIN_CNF_DIR_OUTPUT
            | NRF51_GPIO_PIN_CNF_DRIVE_S0D1;
        break;
    case DEV_PIN_OPENSOURCE:
        ret = NRF51_GPIO_PIN_CNF_DIR_OUTPUT
            | NRF51_GPIO_PIN_CNF_DRIVE_D0S1;
        break;
    case DEV_PIN_OPENDRAIN_PULLUP:
        ret = NRF51_GPIO_PIN_CNF_DIR_OUTPUT
            | NRF51_GPIO_PIN_CNF_DRIVE_S0D1
            | NRF51_GPIO_PIN_CNF_PULL_UP;
        break;
    case DEV_PIN_OPENSOURCE_PULLDOWN:
        ret = NRF51_GPIO_PIN_CNF_DIR_OUTPUT
            | NRF51_GPIO_PIN_CNF_DRIVE_D0S1
            | NRF51_GPIO_PIN_CNF_PULL_DOWN;
        break;
    default:
        return -ENOTSUP;
    }

    *nrf_mode = ret;
    return 0;
}

#if defined(CONFIG_DRIVER_NRF51_GPIO_INPUT_RANGE)
static void nrf51_gpio_input_range_update(struct device_s *dev, uint32_t to_update)
{
  struct nrf51_gpio_private_s *pv = dev->drv_pv;

  nrf_it_disable(GPIOTE_ADDR, NRF51_GPIOTE_PORT);

  uint32_t new_state = nrf_reg_get(GPIO_ADDR, NRF51_GPIO_IN);
  uint32_t old_state = new_state ^ to_update;

  dprintk("%s %x %d\n", __FUNCTION__, to_update, nrf_event_check(GPIOTE_ADDR, NRF51_GPIOTE_PORT));

  while (new_state != old_state) {
    to_update = new_state ^ old_state;

    while (to_update) {
      uint8_t pin = __builtin_ctz(to_update);
      uint32_t cnf = nrf_reg_get(GPIO_ADDR, NRF51_GPIO_PIN_CNF(pin))
        & ~NRF51_GPIO_PIN_CNF_SENSE_MASK;
      nrf_reg_set(GPIO_ADDR, NRF51_GPIO_PIN_CNF(pin), cnf);

      if (pv->range_mask & (1 << pin)) {
        if (new_state & (1 << pin))
          cnf |= NRF51_GPIO_PIN_CNF_SENSE_LOW;
        else
          cnf |= NRF51_GPIO_PIN_CNF_SENSE_HIGH;
      }

      to_update &= ~(1 << pin);

      dprintk(" %d %x\n", pin, cnf);

      nrf_reg_set(GPIO_ADDR, NRF51_GPIO_PIN_CNF(pin), cnf);
    }

    old_state = new_state;
    new_state = nrf_reg_get(GPIO_ADDR, NRF51_GPIO_IN);
  }

  if (pv->range_mask) {
    nrf_event_clear(GPIOTE_ADDR, NRF51_GPIOTE_PORT);
    nrf_it_enable(GPIOTE_ADDR, NRF51_GPIOTE_PORT);
  }
}
#endif

static DEV_GPIO_SET_MODE(nrf51_gpio_set_mode)
{
  struct device_s *dev = gpio->dev;
  struct nrf51_gpio_private_s *pv = dev->drv_pv;
  uint32_t nrf_mode;

  if (io_last > GPIO_COUNT)
    return -ERANGE;

  if (nrf51_gpio_mode(mode, &nrf_mode))
    return -ENOTSUP;

  uint32_t m = (endian_le32_na_load(mask) << io_first)
    & ((1 << (io_last + 1)) - ((1 << io_first)));

  LOCK_SPIN_IRQ(&dev->lock);

  for (uint8_t pin = io_first; pin <= io_last; ++pin) {
    if (!((m >> pin) & 1))
      continue;

    dprintk("%s %d %x\n", __FUNCTION__, pin, nrf_mode);

    nrf_reg_set(GPIO_ADDR, NRF51_GPIO_PIN_CNF(pin), nrf_mode);
  }

#if defined(CONFIG_DRIVER_NRF51_GPIO_INPUT_RANGE)
  if (pv->range_mask)
    nrf51_gpio_input_range_update(dev, m & pv->range_mask);
#endif

  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}

static DEV_GPIO_SET_OUTPUT(nrf51_gpio_set_output)
{
  struct device_s *dev = gpio->dev;

  if (io_last > GPIO_COUNT)
    return -ERANGE;

  if (io_first > GPIO_COUNT)
    return -ERANGE;

  uint32_t mask = (1 << (io_last + 1)) - (1 << io_first);
  uint32_t setm = (endian_le32_na_load(set_mask) << io_first) & mask;
  uint32_t clearm = (endian_le32_na_load(clear_mask) << io_first) | ~mask;

  LOCK_SPIN_IRQ(&dev->lock);

  uint32_t out = nrf_reg_get(GPIO_ADDR, NRF51_GPIO_OUT);
  uint32_t next = setm ^ (out & (setm ^ clearm));

  dprintk("%s %d-%d clr %08x set %08x %08x -> %08x\n",
         __FUNCTION__, io_first, io_last, clearm, setm,
         out, next);

  nrf_reg_set(GPIO_ADDR, NRF51_GPIO_OUT, next);

  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}

static DEV_GPIO_GET_INPUT(nrf51_gpio_get_input)
{
  struct device_s *dev = gpio->dev;

  if (io_last > GPIO_COUNT)
    return -ERANGE;

  LOCK_SPIN_IRQ(&dev->lock);

  uint32_t mask = (1 << (io_last - io_first + 1)) - 1;
  uint32_t in = nrf_reg_get(GPIO_ADDR, NRF51_GPIO_IN) >> io_first;
  endian_le32_na_store(data, in & mask);

  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}

#if defined(CONFIG_DRIVER_NRF51_GPIO_INPUT_RANGE)

static DEV_GPIO_INPUT_IRQ_RANGE(nrf51_gpio_input_irq_range)
{
  struct device_s *dev = gpio->dev;
  struct nrf51_gpio_private_s *pv = dev->drv_pv;

  if (ep_id != NRF51_GPIO_RANGE_IRQ_ID)
    return -EINVAL;

  if (io_last > GPIO_COUNT)
    return -ERANGE;

  if (io_first > GPIO_COUNT)
    return -ERANGE;

  if (mode & (DEV_IRQ_SENSE_RISING_EDGE
              | DEV_IRQ_SENSE_ASYNC_RISING_EDGE
              | DEV_IRQ_SENSE_FALLING_EDGE
              | DEV_IRQ_SENSE_ASYNC_FALLING_EDGE))
    return -ENOTSUP;

  uint32_t selected = (endian_le32_na_load(mask) << io_first)
    & ((1 << (io_last + 1)) - ((1 << io_first)));

  dprintk("%s %08x %d\n", __FUNCTION__, selected, mode);

  LOCK_SPIN_IRQ(&dev->lock);

  uint32_t to_update = selected | pv->range_mask;

  pv->range_mask = selected;
  pv->range_mode = mode;
  nrf51_gpio_input_range_update(dev, to_update);

  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}

#endif

#if defined(CONFIG_DRIVER_NRF51_GPIO_ICU) ||  defined(CONFIG_DRIVER_NRF51_GPIO_PWM)

static int8_t gpiote_allocate(struct nrf51_gpio_private_s *pv,
                              uint8_t pin)
{
  for (uint8_t i = 0; i < GPIOTE_COUNT; ++i)
    if (pv->gpiote_pin[i] == pin)
      return i;

  for (uint8_t i = 0; i < GPIOTE_COUNT; ++i) {
    if (pv->gpiote_pin[i] == -1) {
      pv->gpiote_pin[i] = pin;
      return i;
    }
  }

  for (uint8_t i = 0; i < GPIOTE_COUNT; ++i) {
#if defined(CONFIG_DRIVER_NRF51_GPIO_ICU)
    if (pv->irq_out[i].link_count)
      continue;
#endif

    pv->gpiote_pin[i] = pin;
    return i;
  }

  return -1;
}

static DEV_ICU_GET_ENDPOINT(nrf51_gpio_icu_get_endpoint)
{
  struct device_s *dev = accessor->dev;
  struct nrf51_gpio_private_s *pv = dev->drv_pv;

  switch (type) {
  case DEV_IRQ_EP_SINK:
#if defined(CONFIG_DRIVER_NRF51_GPIO_INPUT_RANGE)
    if (id == NRF51_GPIO_RANGE_IRQ_ID)
      return &pv->range_irq_out;
#endif

    if (id >= GPIO_COUNT)
        return NULL;

    int8_t i = gpiote_allocate(pv, id);
    if (i >= 0)
      return pv->irq_out + i;

    return NULL;

  case DEV_IRQ_EP_SOURCE:
    if (id < 1)
      return pv->irq_in;
    break;

  case DEV_IRQ_EP_BYPASS:
    return NULL;
  }

  return NULL;
}

static DEV_ICU_ENABLE_IRQ(nrf51_gpio_icu_enable_irq)
{
  struct device_s *dev = accessor->dev;
  struct nrf51_gpio_private_s *pv = dev->drv_pv;

#if defined(CONFIG_DRIVER_NRF51_GPIO_INPUT_RANGE)
  if (sink == &pv->range_irq_out) {
    nrf_it_enable(GPIOTE_ADDR, NRF51_GPIOTE_PORT);
    return 1;
  }
#endif

  uint_fast8_t sense = src->sense & sink->sense;
  uint_fast8_t te = sink - pv->irq_out;
  uint8_t pin = pv->gpiote_pin[te];

  if (te >= GPIOTE_COUNT)
    return -EINVAL;

  if (pin >= GPIO_COUNT || pin < 0)
    return -EINVAL;

  for (uint8_t i = 0; i < GPIOTE_COUNT; ++i) {
    if (pv->gpiote_pin[i] == pin && i != te) {
      return -EBUSY;
    }
  }

  if (!sense)
    return -EINVAL;

  bool_t up = !!(sense & (DEV_IRQ_SENSE_HIGH_LEVEL
                          | DEV_IRQ_SENSE_RISING_EDGE
                          | DEV_IRQ_SENSE_ASYNC_RISING_EDGE));
  bool_t down = !!(sense & (DEV_IRQ_SENSE_LOW_LEVEL
                            | DEV_IRQ_SENSE_FALLING_EDGE
                            | DEV_IRQ_SENSE_ASYNC_FALLING_EDGE));

  if (up == down)
    return -ENOTSUP;

  uint32_t te_config = NRF51_GPIOTE_CONFIG_MODE_EVENT
    | NRF51_GPIOTE_CONFIG_PSEL(pin);
  uint32_t pin_config = 0
    | NRF51_GPIO_PIN_CNF_DIR_INPUT
    | NRF51_GPIO_PIN_CNF_INPUT_DISCONNECT;

  if (up) {
    te_config |= NRF51_GPIOTE_CONFIG_POLARITY_LOTOHI;
    pin_config |= NRF51_GPIO_PIN_CNF_PULL_DOWN;
  } else {
    te_config |= NRF51_GPIOTE_CONFIG_POLARITY_HITOLO;
    pin_config |= NRF51_GPIO_PIN_CNF_PULL_UP;
  }

  nrf_reg_set(GPIO_ADDR, NRF51_GPIO_PIN_CNF(pin), pin_config);
  nrf_reg_set(GPIOTE_ADDR, NRF51_GPIOTE_CONFIG(te), te_config);
  nrf_it_enable(GPIOTE_ADDR, NRF51_GPIOTE_IN(te));

  return 1;
}

static DEV_ICU_DISABLE_IRQ(nrf51_gpio_icu_disable_irq)
{
  struct nrf51_gpio_private_s *pv = accessor->dev->drv_pv;
  uint_fast8_t te = sink - pv->irq_out;
  uint8_t pin = pv->gpiote_pin[te];

#if defined(CONFIG_DRIVER_NRF51_GPIO_INPUT_RANGE)
  if (sink == &pv->range_irq_out) {
    nrf_it_disable(GPIOTE_ADDR, NRF51_GPIOTE_PORT);
    return;
  }
#endif

  nrf_reg_set(GPIO_ADDR, NRF51_GPIO_PIN_CNF(pin), 0x0);
  nrf_it_disable(GPIOTE_ADDR, NRF51_GPIOTE_IN(te));
  nrf_reg_set(GPIOTE_ADDR, NRF51_GPIOTE_CONFIG(te), NRF51_GPIOTE_CONFIG_MODE_DISABLED);
  pv->gpiote_pin[te] = -1;
}

static DEV_IRQ_EP_PROCESS(nrf51_gpio_process)
{
  struct nrf51_gpio_private_s *pv = ep->dev->drv_pv;
  bool_t evented = 1;

  while (evented) {
    evented = 0;

    for (int8_t te = 0; te < GPIOTE_COUNT; ++te) {
      if (!nrf_event_check(GPIOTE_ADDR, NRF51_GPIOTE_IN(te)))
        continue;

      nrf_event_clear(GPIOTE_ADDR, NRF51_GPIOTE_IN(te));

      struct dev_irq_ep_s *sink = pv->irq_out + te;
      sink->process(sink, 0);
      evented = 1;
    }

#if defined(CONFIG_DRIVER_NRF51_GPIO_INPUT_RANGE)
    if (nrf_event_check(GPIOTE_ADDR, NRF51_GPIOTE_PORT)) {
      nrf_event_clear(GPIOTE_ADDR, NRF51_GPIOTE_PORT);
      dprintk("%s port %08x %d\n", __FUNCTION__, pv->range_mask, pv->range_mode);

      nrf51_gpio_input_range_update(ep->dev, pv->range_mask);

      pv->range_irq_out.process(&pv->range_irq_out, 0);

      evented = 1;
    }
#endif
  }
}

#endif /* ICU */

#if defined(CONFIG_DRIVER_NRF51_GPIO_PWM)
static DEV_PWM_CONFIG(nrf51_gpio_pwm_config)
{

}
#endif /* PWM */

static DEV_IOMUX_SETUP(nrf51_gpio_iomux_setup)
{
  if (io_id >= GPIO_COUNT)
    return -ERANGE;

  uint32_t nrf_mode;
  if (nrf51_gpio_mode(dir, &nrf_mode))
    return -ENOTSUP;

  nrf_reg_set(GPIO_ADDR, NRF51_GPIO_PIN_CNF(io_id), nrf_mode);

  return 0;
}

#define nrf51_gpio_request dev_gpio_request_async_to_sync

static DEV_INIT(nrf51_gpio_init);
static DEV_CLEANUP(nrf51_gpio_cleanup);
#define nrf51_gpio_use dev_use_generic

DRIVER_DECLARE(nrf51_gpio_drv, "nRF51 GPIO"
#if defined(CONFIG_DRIVER_NRF51_GPIO_ICU)
               ",ICU"
#endif
#if defined(CONFIG_DRIVER_NRF51_GPIO_INPUT_RANGE)
               ",RANGE"
#endif
#if defined(CONFIG_DRIVER_NRF51_GPIO_PWM)
               ",PWM"
#endif
               , nrf51_gpio,
#if defined(CONFIG_DRIVER_NRF51_GPIO_ICU)
               DRIVER_ICU_METHODS(nrf51_gpio_icu),
#endif
#if defined(CONFIG_DRIVER_NRF51_GPIO_PWM)
               DRIVER_PWM_METHODS(nrf51_gpio_pwm),
#endif
               DRIVER_GPIO_METHODS(nrf51_gpio),
               DRIVER_IOMUX_METHODS(nrf51_gpio_iomux));

static DEV_INIT(nrf51_gpio_init)
{
  struct nrf51_gpio_private_s *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  __unused__ uintptr_t addr = 0;
  assert(!device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL) && GPIO_ADDR == addr);

#if defined(CONFIG_DRIVER_NRF51_GPIO_ICU) || defined(CONFIG_DRIVER_NRF51_GPIO_PWM)
  pv = mem_alloc(sizeof(*pv), mem_scope_sys);

  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  assert(!device_res_get_uint(dev, DEV_RES_MEM, 1, &addr, NULL) && GPIOTE_ADDR == addr);

  nrf_reg_set(GPIOTE_ADDR, NRF51_GPIOTE_POWER, 0);
  for (uint16_t i = 0; i < 32; ++i)
    asm volatile("");

  nrf_reg_set(GPIOTE_ADDR, NRF51_GPIOTE_POWER, 1);
  for (uint16_t i = 0; i < 32; ++i)
    asm volatile("");

  for (int8_t i = 0; i < GPIOTE_COUNT; ++i) {
    nrf_it_disable(GPIOTE_ADDR, NRF51_GPIOTE_IN(i));
    nrf_reg_set(GPIOTE_ADDR, NRF51_GPIOTE_CONFIG(i),
                NRF51_GPIOTE_CONFIG_MODE_DISABLED);
    pv->gpiote_pin[i] = -1;
  }
#endif



#if defined(CONFIG_DRIVER_NRF51_GPIO_PWM)
  for (uint8_t i = 0; i < PWM_COUNT; ++i) {
    nrf51_ppi_alloc(pv->pwm[i].ppi, 2);

    nrf51_ppi_setup(NRF51_PPI_ADDR, pv->pwm[i].ppi[0],
                    pv->pwm_timer_addr, NRF51_TIMER_COMPARE(OVERFLOW),
                    GPIOTE_ADDR, 1);

    nrf51_ppi_setup(NRF51_PPI_ADDR, pv->pwm[i].ppi[0],
                    pv->pwm_timer_addr, NRF51_TIMER_COMPARE(OVERFLOW),
                    GPIOTE_ADDR, 1);
  }
#endif

#if defined(CONFIG_DRIVER_NRF51_GPIO_ICU)
  device_irq_source_init(dev, pv->irq_in, 1,
                         &nrf51_gpio_process,
                         DEV_IRQ_SENSE_HIGH_LEVEL);

  if (device_irq_source_link(dev, pv->irq_in, 1, -1))
    goto err_mem;

  device_irq_sink_init(dev, pv->irq_out,
                       GPIOTE_COUNT,
                       DEV_IRQ_SENSE_HIGH_LEVEL
                       | DEV_IRQ_SENSE_LOW_LEVEL
                       | DEV_IRQ_SENSE_RISING_EDGE
                       | DEV_IRQ_SENSE_FALLING_EDGE);

# if defined(CONFIG_DRIVER_NRF51_GPIO_INPUT_RANGE)
  device_irq_sink_init(dev, &pv->range_irq_out, 1, DEV_IRQ_SENSE_ANY_EDGE);
# endif
#endif

  for (uint8_t pin = 0; pin <= 31; ++pin)
    nrf_reg_set(GPIO_ADDR, NRF51_GPIO_PIN_CNF(pin), 0
                | NRF51_GPIO_PIN_CNF_DIR_INPUT
                | NRF51_GPIO_PIN_CNF_INPUT_DISCONNECT);

  dev->drv = &nrf51_gpio_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;
  return 0;

 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(nrf51_gpio_cleanup)
{
#if defined(CONFIG_DRIVER_NRF51_GPIO_ICU)
  struct nrf51_gpio_private_s  *pv = dev->drv_pv;

  for (int8_t i = 0; i < GPIOTE_COUNT; ++i)
    nrf_it_disable(GPIOTE_ADDR, NRF51_GPIOTE_IN(i));

  device_irq_source_unlink(dev, pv->irq_in, 1);

  device_irq_sink_unlink(dev, pv->irq_out,
                         GPIOTE_COUNT);

# if defined(CONFIG_DRIVER_NRF51_GPIO_PWM)
  for (uint8_t i = 0; i < PWM_COUNT; ++i)
    nrf51_ppi_free(pv->pwm[i].ppi, 2);
# endif

# if defined(CONFIG_DRIVER_NRF51_GPIO_INPUT_RANGE)
  device_irq_sink_unlink(dev, &pv->range_irq_out, 1);
# endif
#endif

#if defined(CONFIG_DRIVER_NRF51_GPIO_ICU) || defined(CONFIG_DRIVER_NRF51_GPIO_PWM)
  mem_free(pv);
#endif
}
