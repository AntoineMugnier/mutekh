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
    Copyright (c) 2014 Julien Peeters <contact@julienpeeters.net>

*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/device.h>
#include <device/driver.h>
#include <device/resources.h>
#include <device/irq.h>
#include <device/class/gpio.h>
#include <device/class/icu.h>
#include <device/class/iomux.h>

#include <arch/stm32_exti.h>
#include <arch/stm32_gpio.h>
#include <arch/stm32_rcc.h>
#include <arch/stm32_syscfg.h>

#include <cpp/device/helpers.h>
#include <arch/stm32_memory_map.h>


#define STM32_GPIO_BANK_WIDTH       0x400
#define STM32_GPIO_BANK_SIZE        16
#define STM32_GPIO_BANK_COUNT       5
#define STM32_GPIO_MAX_ID                                \
  (STM32_GPIO_BANK_SIZE * STM32_GPIO_BANK_COUNT - 1) \
/**/
#define STM32_GPIO_IRQ_SRC_COUNT    7

struct stm32_gpio_context_s
{
  /* address of the device. */
  uintptr_t             addr;

#if defined(CONFIG_DRIVER_STM32_GPIO_ICU)
  struct dev_irq_ep_s   sink[CONFIG_DRIVER_STM32_GPIO_IRQ_COUNT];

  /* This specifies which bank is selected for each interrupt line. A
     value of -1 means that no bank is currently bound to an
     interrupt. */
  struct {
    int8_t bank:3;
    bool_t enabled:1;
  }                     irq[CONFIG_DRIVER_EFM32_GPIO_IRQ_COUNT];

  struct dev_irq_ep_s   src[STM32_GPIO_IRQ_SRC_COUNT];
#endif
};


/********************************* GPIO class. **********/

#define STM32_GPIO_MODE_INPUT       1
#define STM32_GPIO_MODE_OUTPUT      2
#define STM32_GPIO_MODE_ANALOG      4
#define STM32_GPIO_MODE_PUSHPULL    8
#define STM32_GPIO_MODE_OPENDRAIN   16
#define STM32_GPIO_MODE_PULLUP      32
#define STM32_GPIO_MODE_PULLDOWN    64

static
error_t stm32_gpio_gpio_make_mode(enum dev_pin_driving_e mode,
                                  uint8_t                *bf)
{
  *bf = 0;
  switch (mode)
    {
    default:
      return -ENOTSUP;

    case DEV_PIN_DISABLED:
      break;

    case DEV_PIN_PUSHPULL:
      *bf = STM32_GPIO_MODE_OUTPUT | STM32_GPIO_MODE_PUSHPULL;
      break;

    case DEV_PIN_INPUT:
      *bf = STM32_GPIO_MODE_INPUT;
      break;

    case DEV_PIN_INPUT_PULLUP:
      *bf = STM32_GPIO_MODE_INPUT | STM32_GPIO_MODE_PULLUP;
      break;

    case DEV_PIN_OPENDRAIN:
      *bf = STM32_GPIO_MODE_OUTPUT | STM32_GPIO_MODE_OPENDRAIN;
      break;

    case DEV_PIN_OPENSOURCE:
      *bf = STM32_GPIO_MODE_OUTPUT | STM32_GPIO_MODE_OPENDRAIN;
      break;

    case DEV_PIN_OPENDRAIN_PULLUP:
      *bf = STM32_GPIO_MODE_OUTPUT |
        STM32_GPIO_MODE_OPENDRAIN  |
        STM32_GPIO_MODE_PULLUP;
      break;

    case DEV_PIN_OPENSOURCE_PULLDOWN:
      *bf = STM32_GPIO_MODE_OUTPUT |
        STM32_GPIO_MODE_OPENDRAIN  |
        STM32_GPIO_MODE_PULLDOWN;
      break;
    }
  return 0;
}

static
void stm32_gpio_gpio_apply_mode(struct device_s *dev,
                                gpio_id_t       iopin,
                                uint8_t         bf)
{
  struct stm32_gpio_context_s *pv  = dev->drv_pv;

  uint8_t bank           = iopin / STM32_GPIO_BANK_SIZE;
  uint8_t io_in_bank     = iopin % STM32_GPIO_BANK_SIZE;
  uintptr_t const bkaddr = pv->addr + (bank * STM32_GPIO_BANK_WIDTH);

  /* Input/output. */
  if (bf & STM32_GPIO_MODE_OUTPUT)
    DEVICE_REG_FIELD_IDX_UPDATE_DEV(
      GPIO,
      bkaddr,
      MODER,
      MODE,
      io_in_bank,
      OUTPUT
    );
  else if (bf & STM32_GPIO_MODE_INPUT)
    DEVICE_REG_FIELD_IDX_UPDATE_DEV(
      GPIO,
      bkaddr,
      MODER,
      MODE,
      io_in_bank,
      INPUT
    );

  /* Pushpull/open-drain. */
  if (bf & STM32_GPIO_MODE_PUSHPULL)
    DEVICE_REG_FIELD_IDX_UPDATE_DEV(
      GPIO,
      bkaddr,
      OTYPER,
      OT,
      io_in_bank,
      PUSHPULL
    );
  else if (bf & STM32_GPIO_MODE_OPENDRAIN)
    DEVICE_REG_FIELD_IDX_UPDATE_DEV(
      GPIO,
      bkaddr,
      OTYPER,
      OT,
      io_in_bank,
      OPEN
    );

  /* Pull-up/pull-down. */
  if (bf & STM32_GPIO_MODE_PULLUP)
    DEVICE_REG_FIELD_IDX_UPDATE_DEV(
      GPIO,
      bkaddr,
      PUPDR,
      PUPD,
      io_in_bank,
      PULLUP
    );
  else if (bf & STM32_GPIO_MODE_PULLDOWN)
    DEVICE_REG_FIELD_IDX_UPDATE_DEV(
      GPIO,
      bkaddr,
      PUPDR,
      PUPD,
      io_in_bank,
      PULLDOWN
    );

  /* set gpio speed. */
  extern uint32_t stm32f4xx_clock_freq_ahb1;
  if (stm32f4xx_clock_freq_ahb1 >= 80000000)
    DEVICE_REG_FIELD_IDX_UPDATE_DEV(
      GPIO,
      bkaddr,
      OSPEEDR,
      OSPEED,
      io_in_bank,
      HIGH
    );
  else if (stm32f4xx_clock_freq_ahb1 >= 50000000)
    DEVICE_REG_FIELD_IDX_UPDATE_DEV(
      GPIO,
      bkaddr,
      OSPEEDR,
      OSPEED,
      io_in_bank,
      FAST
    );
  else if (stm32f4xx_clock_freq_ahb1 >= 25000000)
    DEVICE_REG_FIELD_IDX_UPDATE_DEV(
      GPIO,
      bkaddr,
      OSPEEDR,
      OSPEED,
      io_in_bank,
      MEDIUM
    );
  else
    DEVICE_REG_FIELD_IDX_UPDATE_DEV(
      GPIO,
      bkaddr,
      OSPEEDR,
      OSPEED,
      io_in_bank,
      LOW
    );
}

static
DEVGPIO_SET_MODE(stm32_gpio_gpio_set_mode)
{
  struct device_s *dev = gpio->dev;
  uint8_t         msk_idx = 0, bf;

  if (io_first > io_last || io_last > STM32_GPIO_MAX_ID)
    return -ERANGE;

  if (stm32_gpio_gpio_make_mode(mode, &bf))
    return -ENOTSUP;

  LOCK_SPIN_IRQ(&dev->lock);

  for (; io_first <= io_last && msk_idx < 64; ++io_first)
    {
      if ((mask[msk_idx/8] >> (msk_idx % 8)) & 0x1)
        {
          stm32_gpio_gpio_apply_mode(dev, io_first, bf);
        }
      ++msk_idx;
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}

static
DEVGPIO_SET_OUTPUT(stm32_gpio_gpio_set_output)
{
  struct device_s             *dev = gpio->dev;
  struct stm32_gpio_context_s *pv  = dev->drv_pv;
  uint8_t                     msk_idx = 0;

  if (io_first > io_last || io_last > STM32_GPIO_MAX_ID)
    return -ERANGE;

  LOCK_SPIN_IRQ(&dev->lock);

  for (; io_first <= io_last && msk_idx < 64; ++io_first)
    {
      uint8_t bank           = io_first / STM32_GPIO_BANK_SIZE;
      uint8_t io_in_bank     = io_first % STM32_GPIO_BANK_SIZE;
      uintptr_t const bkaddr = pv->addr + (bank * STM32_GPIO_BANK_WIDTH);

      uint8_t sval = (set_mask[msk_idx/8] >> (msk_idx % 8)) & 0x1;
      uint8_t cval = (clear_mask[msk_idx/8] >> (msk_idx % 8)) & 0x1;

      ++msk_idx;

      /* sval cval concatenated. */
      switch ((sval << 1) | cval)
      {
      default: break;

      /* clear */
      case 0 /* 00 */:
        DEVICE_REG_FIELD_IDX_SET_DEV(GPIO, bkaddr, BSRR, BR, io_in_bank);
        break;

      /* unchanged */
      case 1 /* 01 */:
        break;

      /* toggle */
      case 2 /* 10 */: {
        uint32_t register odrval = DEVICE_REG_FIELD_IDX_VALUE_DEV(
          GPIO,
          bkaddr,
          ODR,
          OD,
          io_in_bank
        );
        DEVICE_REG_FIELD_IDX_UPDATE_DEV(
          GPIO,
          bkaddr,
          BSRR,
          BR,
          io_in_bank,
          ~odrval
        );
        break;
      }

      /* set */
      case 3 /* 11 */:
        DEVICE_REG_FIELD_IDX_SET_DEV(GPIO, bkaddr, BSRR, BS, io_in_bank);
        break;
      }
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}

static
DEVGPIO_GET_INPUT(stm32_gpio_gpio_get_input)
{
  struct device_s             *dev = gpio->dev;
  struct stm32_gpio_context_s *pv  = dev->drv_pv;

  if (io_first > io_last || io_last > STM32_GPIO_MAX_ID)
    return -ERANGE;

  LOCK_SPIN_IRQ(&dev->lock);

  for (; io_first <= io_last; io_first += 32)
    {
      uint8_t bank           = io_first / STM32_GPIO_BANK_SIZE;
      uintptr_t const bkaddr = pv->addr + (bank * STM32_GPIO_BANK_WIDTH);

      uint32_t register value = DEVICE_REG_VALUE_DEV(GPIO, bkaddr, IDR);
      *(uint32_t*)data = value >> io_first;

      data += 4;
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}


static const struct driver_gpio_s stm32_gpio_gpio_cls =
{
  .class_       = DRIVER_CLASS_GPIO,
  .f_set_mode   = &stm32_gpio_gpio_set_mode,
  .f_set_output = &stm32_gpio_gpio_set_output,
  .f_get_input  = &stm32_gpio_gpio_get_input,
  .f_request     = devgpio_request_async_to_sync,
};


/********************************* IOMUX class. **********/

static
DEVIOMUX_SETUP(stm32_gpio_iomux_setup)
{
  struct device_s             *dev = imdev->dev;
  struct stm32_gpio_context_s *pv  = dev->drv_pv;
  uint8_t                     bf;

  uint8_t bank           = io_id / STM32_GPIO_BANK_SIZE;
  uint8_t io_in_bank     = io_id % STM32_GPIO_BANK_SIZE;
  uintptr_t const bkaddr = pv->addr + (bank * STM32_GPIO_BANK_WIDTH);

  if (io_id > STM32_GPIO_MAX_ID)
    return -ERANGE;

  if (stm32_gpio_gpio_make_mode(dir, &bf))
    return -ENOTSUP;

  /* alternate functions between 0 and 15. */
  if (mux > 15)
    return -EINVAL;

  /* configure the alternate function (number in mux argument). */

  DEVICE_REG_FIELD_IDX_UPDATE_DEV(
    GPIO,
    bkaddr,
    MODER,
    MODE,
    io_in_bank,
    ALT
  );

  if (io_in_bank > 7)
    DEVICE_REG_FIELD_IDX_UPDATE_DEV(
      GPIO,
      bkaddr,
      AFRH,
      AF,
      (io_in_bank - 8),
      mux
    );
  else
    DEVICE_REG_FIELD_IDX_UPDATE_DEV(
      GPIO,
      bkaddr,
      AFRL,
      AF,
      io_in_bank,
      mux
    );

  /* remove input/output in bitfield as we use here alternate function. */
  bf &= ~(STM32_GPIO_MODE_INPUT | STM32_GPIO_MODE_OUTPUT);
  stm32_gpio_gpio_apply_mode(dev, io_id, bf);

  return 0;
}


static const struct driver_iomux_s stm32_gpio_iomux_cls =
{
  .class_  = DRIVER_CLASS_IOMUX,
  .f_setup = &stm32_gpio_iomux_setup,
};


/********************************* ICU class. **********/

#if defined(CONFIG_DRIVER_STM32_GPIO_ICU)

static
uint_fast8_t stm32_gpio_icu_source_of_sink(uint_fast8_t icu_sink_id)
{
  switch (icu_sink_id)
    {
    default:
      assert(0 && "non reachable.");
      return 0;

    case 0 ... 4:
      return icu_sink_id;

    case 5 ... 9:
      return STM32_GPIO_IRQ_SRC_COUNT-2;

    case 10 ... STM32_GPIO_BANK_SIZE-1:
      return STM32_GPIO_IRQ_SRC_COUNT-1;
    }
}

static
DEVICU_GET_ENDPOINT(stm32_gpio_icu_get_endpoint)
{
  struct device_s             *dev = idev->dev;
  struct stm32_gpio_context_s *pv = dev->drv_pv;

  switch (type)
    {
    default:
      return NULL;

    case DEV_IRQ_EP_SINK: {
      uint8_t bank       = id / STM32_GPIO_BANK_SIZE;
      uint8_t io_in_bank = id % STM32_GPIO_BANK_SIZE;

      if (io_in_bank >= CONFIG_DRIVER_STM32_GPIO_IRQ_COUNT || bank >= 5)
        return NULL;

      struct dev_irq_ep_s *ep = &pv->sink[io_in_bank];

      /* We actually keep only one end-point object per line for all
         banks. We have to keep track of which bank the end-point is
         associated to. */
      if (ep->link_count == 0)
        {
          pv->irq[io_in_bank].bank    = bank;
          pv->irq[io_in_bank].enabled = 0;
          ep->sense = DEV_IRQ_SENSE_FALLING_EDGE | DEV_IRQ_SENSE_RISING_EDGE;
        }
      else if (pv->irq[io_in_bank].bank != bank)
        return NULL;

      return ep;
    }

    case DEV_IRQ_EP_SOURCE: {
      uint_fast8_t icu_src_id = stm32_gpio_icu_source_of_sink(id);
      if (id < STM32_GPIO_IRQ_SRC_COUNT)
        return &pv->src[icu_src_id];
      return NULL;
    }
    }
}

static
DEVICU_ENABLE_IRQ(stm32_gpio_icu_enable_irq)
{
  struct device_s             *dev        = idev->dev;
  struct stm32_gpio_context_s *pv         = dev->drv_pv;
  uint_fast8_t                icu_sink_id = sink - pv->sink;

  if (irq_id > 0)
    {
      printk(
        "STM32 GPIO %p: single wire IRQ must use 0 as logical"
          " IRQ id for %p device.\n",
        dev,
        dev_ep->dev
      );
      return 0;
    }

  /* take the common sensing configuration between source and sink. */
  uint_fast8_t sense = src->sense & sink->sense;
  if ((sense & (DEV_IRQ_SENSE_RISING_EDGE | DEV_IRQ_SENSE_FALLING_EDGE)) == 0)
    return 0;

  /* enable sink irq and activate corresponding source irq. */
  uint_fast8_t icu_src_id = stm32_gpio_icu_source_of_sink(icu_sink_id);

  if (!device_icu_irq_enable(&pv->src[icu_src_id], 0, NULL, dev_ep))
    {
      printk("STM32 GPIO: source IRQ end-point cannot relay"
             " interrupt for %p device.\n",
             dev_ep);
      return 0;
    }

  /* if more than one mode is left, give priority to rising edge event. */
  sense &= ~(sense - 1);
  src->sense = sink->sense = sense;

  /* if IRQ is not yet enabled, configure it and enable it. */
  uint8_t io_in_bank = icu_sink_id % STM32_GPIO_BANK_SIZE;
  if (!pv->irq[io_in_bank].enabled)
    {
      uint8_t bank          = pv->irq[io_in_bank].bank;

      /* select sink bank. */
      DEVICE_REG_IDX_FIELD_IDX_UPDATE(
        SYSCFG, ,
        EXTICR,
        io_in_bank / 4,
        EXTI,
        io_in_bank % 4,
        bank
      );

      /* select polarity of interrupt edge. */
      if (sense & DEV_IRQ_SENSE_RISING_EDGE)
        {
          DEVICE_REG_FIELD_IDX_SET(EXTI, , RTSR, TR, io_in_bank);
          DEVICE_REG_FIELD_IDX_CLR(EXTI, , FTSR, TR, io_in_bank);
        }
      else if (sense & DEV_IRQ_SENSE_FALLING_EDGE)
        {
          DEVICE_REG_FIELD_IDX_SET(EXTI, , FTSR, TR, io_in_bank);
          DEVICE_REG_FIELD_IDX_CLR(EXTI, , RTSR, TR, io_in_bank);
        }
      else
        assert(0 && "unsupported irq sensing method");

      /* force pin to input mode. */
      stm32_gpio_gpio_apply_mode(dev, icu_sink_id, STM32_GPIO_MODE_INPUT);

      /* clear interrupt. */
      DEVICE_REG_FIELD_IDX_SET(EXTI, , PR, PR, io_in_bank);

      /* enable interrupt. */
      DEVICE_REG_FIELD_IDX_SET(EXTI, , IMR, MR, io_in_bank);

      pv->irq[io_in_bank].enabled = 1;
    }

  return 1;
}

static
DEVICU_DISABLE_IRQ(stm32_gpio_icu_disable_irq)
{
  struct device_s             *dev        = idev->dev;
  struct stm32_gpio_context_s *pv         = dev->drv_pv;
  uint_fast8_t                icu_sink_id = sink - pv->sink;

  uint_fast8_t io_in_bank = icu_sink_id % STM32_GPIO_BANK_SIZE;

  /* disable interrupt. */
  DEVICE_REG_FIELD_IDX_CLR(EXTI, , IMR, MR, io_in_bank);
}

static const struct driver_icu_s stm32_gpio_icu_cls =
  {
    .class_         = DRIVER_CLASS_ICU,
    .f_get_endpoint = &stm32_gpio_icu_get_endpoint,
    .f_enable_irq   = &stm32_gpio_icu_enable_irq,
    .f_disable_irq  = &stm32_gpio_icu_disable_irq,
  };

static
DEV_IRQ_EP_PROCESS(stm32_gpio_irq)
{
  struct device_s             *dev = ep->dev;
  struct stm32_gpio_context_s *pv  = dev->drv_pv;

  while (1)
    {
      /* check for pending interrupts. */
      uint32_t pending = DEVICE_REG_VALUE(EXTI, , PR);
      if (pending == 0)
        break;

      /* clear pending interrupts: write '1' clears the interrupt. */
      DEVICE_REG_UPDATE(EXTI, , PR, pending);

      while (pending != 0)
        {
          uint_fast8_t          icu_sink_id = __builtin_ctz(pending);
          struct dev_irq_ep_s   *sink       = &pv->sink[icu_sink_id];

          assert(icu_sink_id < CONFIG_DRIVER_STM32_GPIO_IRQ_COUNT);

          /* process interrupt. */
          sink->process(sink, id);

          /* mark interrupt as processed. */
          pending ^= 1 << icu_sink_id;
        }
    }
}

#endif

/********************************* DRIVER */

static DEV_INIT(stm32_gpio_init);
static DEV_CLEANUP(stm32_gpio_cleanup);

const struct driver_s stm32_gpio_drv =
  {
    .desc      = "STM32 GPIO",
    .f_init    = &stm32_gpio_init,
    .f_cleanup = &stm32_gpio_cleanup,
    .classes   = {
      &stm32_gpio_gpio_cls,
      &stm32_gpio_iomux_cls,
#if defined(CONFIG_DRIVER_STM32_GPIO_ICU)
      &stm32_gpio_icu_cls,
#endif
      0
    },
  };

REGISTER_DRIVER(stm32_gpio_drv);

static
DEV_INIT(stm32_gpio_init)
{
  struct stm32_gpio_context_s *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));
  dev->drv_pv = pv;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

  /* enable clock gating for gpio A..E. */
  DEVICE_REG_UPDATE(RCC, , AHB1ENR, 0x1f);

#if defined(CONFIG_DRIVER_STM32_GPIO_ICU)
  /* enable clock gating for SYSCFG. */
  DEVICE_REG_FIELD_SET(RCC, , APB2ENR, SYSCFGEN);

  device_irq_source_init(
    dev,
    pv->src,
    STM32_GPIO_IRQ_SRC_COUNT,
    &stm32_gpio_irq,
    DEV_IRQ_SENSE_HIGH_LEVEL
  );

  if (device_irq_source_link(dev, pv->src, STM32_GPIO_IRQ_SRC_COUNT, 0))
    goto err_mem;

  device_irq_sink_init(
    dev,
    pv->sink,
    CONFIG_DRIVER_STM32_GPIO_IRQ_COUNT,
    DEV_IRQ_SENSE_FALLING_EDGE | DEV_IRQ_SENSE_RISING_EDGE
  );
#endif

  dev->drv = &stm32_gpio_drv;

  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;

err_mem:
  mem_free(pv);
  return -1;
}

static
DEV_CLEANUP(stm32_gpio_cleanup)
{
  struct stm32_gpio_context_s *pv = dev->drv_pv;

  /* disable clock gating for gpio A..E. */
  DEVICE_REG_UPDATE(RCC, , AHB1ENR, 0x0);

#if defined(CONFIG_DRIVER_STM32_GPIO_ICU)
  /* disable clock gating for SYSCFG. */
  DEVICE_REG_FIELD_CLR(RCC, , APB2ENR, SYSCFGEN);

  device_irq_source_unlink(dev, pv->src, STM32_GPIO_IRQ_SRC_COUNT);
  device_irq_sink_unlink(dev, pv->sink, CONFIG_DRIVER_STM32_GPIO_IRQ_COUNT);
#endif

  mem_free(pv);
}

