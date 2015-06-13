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

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/device.h>
#include <device/driver.h>
#include <device/resources.h>
#include <device/class/gpio.h>
#include <device/class/icu.h>
#include <device/class/iomux.h>

#include <arch/stm32f1xx_gpio.h>

#include <cpp/device/helpers.h>
#include <arch/stm32_memory_map.h>


#define STM32_GPIO_BANK_SIZE    16
#define STM32_GPIO_BANK_WIDTH   0x400

enum stm32_gpio_mode_e
{
  STM32_GPIO_MODE_OUPUT      = (1<<0),
  STM32_GPIO_MODE_INPUT      = (1<<1),
  STM32_GPIO_MODE_PUSH_PULL  = (1<<2),
  STM32_GPIO_MODE_OPEN_DRAIN = (1<<3),
  STM32_GPIO_MODE_PULL_UP    = (1<<4),
  STM32_GPIO_MODE_PULL_DOWN  = (1<<5),
};

enum stm32_gpio_speed_e
{
    /* > 0 MHz */
    STM32_GPIO_SPEED_LOW,
    /* >= 10 MHz */
    STM32_GPIO_SPEED_MED,
    /* >= 50 MHz */
    STM32_GPIO_SPEED_HIGH,
};

struct stm32_gpio_private_s
{
  uintptr_t addr;
};

static
error_t stm32_gpio_prepare(
  enum dev_pin_driving_e mode,
  uint8_t                *value)
{
  *value = 0;

  switch (mode)
    {
    default:
      return -ENOTSUP;

    case DEV_PIN_DISABLED:
      break;

    case DEV_PIN_PUSHPULL:
      *value = STM32_GPIO_MODE_OUTPUT | STM32_GPIO_MODE_PUSHPULL;
      break;

    case DEV_PIN_OPENDRAIN:
      *value = STM32_GPIO_MODE_OUTPUT | STM32_GPIO_MODE_OPENDRAIN;
      break;

    case DEV_PIN_INPUT:
      *value = STM32_GPIO_MODE_INPUT;
      break;

    case DEV_PIN_INPUT_PULLUP:
      *value = STM32_GPIO_MODE_INPUT | STM32_GPIO_MODE_PULLUP;
      break;

    case DEV_PIN_INPUT_PULLDOWN:
      *value = STM32_GPIO_MODE_INPUT | STM32_GPIO_MODE_PULLDOWN;
      break;
    }

  return 0;
}

static
void stm32_gpio_gpio_apply_general(
  struct device_s         *dev,
  gpio_id_t               iopin,
  uint8_t                 bf
)
{
  struct stm32_gpio_private_s *pv = dev->drv_pv;

  uint8_t bank           = iopin / STM32_GPIO_BANK_SIZE;
  uint8_t io_in_bank     = iopin % STM32_GPIO_BANK_SIZE;
  uintptr_t const bkaddr = pv->addr + (bank * STM32_GPIO_BANK_WIDTH);

  uint32_t regval = 0;

  /* Input/output. */
  if (value & STM32_GPIO_MODE_OUTPUT)
    {
      DEVICE_REG_FIELD_IDX_UPDATE_VAR(GPIO, CRL, MODE, io_in_bank,
        OUTPUT_50MHZ, regval);

      /* Pushpull/open-drain. */
      if (bf & STM32_GPIO_MODE_PUSHPULL)
        DEVICE_REG_FIELD_IDX_UPDATE_VAR(GPIO, CRL, CNF, io_in_bank,
          GENERAL_PUSH_PULL, regval);

      else if (bf & STM32_GPIO_MODE_OPENDRAIN)
        DEVICE_REG_FIELD_IDX_UPDATE_VAR(GPIO, CRL, CNF, io_in_bank,
          GENERAL_OPEN_DRAIN, regval);

      if ( io_in_bank <= 7 )
        DEVICE_REG_UPDATE_DEV(GPIO, bkaddr, CRL, regval);
      else
        DEVICE_REG_UPDATE_DEV(GPIO, bkaddr, CRH, regval);
    }
  else if (bf & STM32_GPIO_MODE_INPUT)
    {
      DEVICE_REG_FIELD_IDX_UPDATE_VAR(GPIO, CRL, MODE, io_in_bank, INPUT,
        regval);

      DEVICE_REG_FIELD_IDX_UPDATE_VAR(GPIO, CRL, CNF, io_in_bank,
        PULL_UP_PULL_DOWN, regval);

      if ( io_in_bank <= 7 )
        DEVICE_REG_UPDATE_DEV(GPIO, bkaddr, CRL, regval);
      else
        DEVICE_REG_UPDATE_DEV(GPIO, bkaddr, CRH, regval);

      /* Pull-up/pull-down. */
      if (bf & STM32_GPIO_MODE_PULLUP)
        DEVICE_REG_FIELD_IDX_SET_DEV(GPIO, bkaddr, ODR, io_in_bank);
      else if (bf & STM32_GPIO_MODE_PULLDOWN)
        DEVICE_REG_FIELD_IDX_CLR_DEV(GPIO, bkaddr, ODR, io_in_bank);
    }
}

static
DEV_GPIO_SET_MODE(stm32_gpio_gpio_set_mode)
{
  struct device_s *dev = gpio->dev;
  uint8_t         msk_idx = 0, bf;

  if (io_first > io_last || io_last > STM32_GPIO_MAX_ID)
    return -ERANGE;

  if (stm32_gpio_gpio_prepare(mode, &bf))
    return -ENOTSUP;

  LOCK_SPIN_IRQ(&dev->lock);

  for (; io_first <= io_last && msk_idx < 64; ++io_first)
    {
      if ((mask[msk_idx/8] >> (msk_idx % 8)) & 0x1)
        {
          stm32_gpio_gpio_apply_general(dev, io_first, bf);
        }
      ++msk_idx;
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}

static
DEV_GPIO_SET_OUTPUT(stm32_gpio_gpio_set_output)
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
        uint32_t register oldval = DEVICE_REG_FIELD_IDX_VALUE_DEV(
          GPIO,
          bkaddr,
          ODR,
          OD,
          io_in_bank
        );
        DEVICE_REG_FIELD_IDX_UPDATE_DEV(GPIO, bkaddr, ODR, OD, io_in_bank,
          oldval ^ 0x1);
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
DEV_GPIO_GET_INPUT(stm32_gpio_gpio_get_input)
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

static const struct driver_gpio_s stm32_gpio_gpio_drv =
{
  .class_        = DRIVER_CLASS_GPIO,
  .f_set_mode    = &stm32_gpio_gpio_set_mode,
  .f_set_output  = &stm32_gpio_gpio_set_output,
  .f_get_input   = &stm32_gpio_gpio_get_input,
  .f_request     = dev_gpio_request_async_to_sync,
};

/********************************* IOMUX class. **********/

static
DEV_IOMUX_SETUP(stm32_gpio_iomux_setup)
{
  struct device_s             *dev = accessor->dev;
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


static const struct driver_iomux_s stm32_gpio_iomux_drv =
{
  .class_  = DRIVER_CLASS_IOMUX,
  .f_setup = &stm32_gpio_iomux_setup,
};

/********************************* DRIVER */

static DEV_INIT(stm32_gpio_init);
static DEV_CLEANUP(stm32_gpio_cleanup);

const struct driver_s stm32_gpio_drv =
  {
    .desc      = "STM32 GPIO",
    .f_init    = &stm32_gpio_init,
    .f_cleanup = &stm32_gpio_cleanup,
    .classes   = {
      &stm32_gpio_gpio_drv,
      &stm32_gpio_iomux_drv,
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

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

  /* enable clock gating for gpio A..E. */
  DEVICE_REG_UPDATE(RCC, , APB2ENR, 0xfd);

  dev->drv_pv = pv;
  dev->drv    = &stm32_gpio_drv;
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
  DEVICE_REG_UPDATE(RCC, , APB2ENR, ~0xfd);

#if defined(CONFIG_DRIVER_STM32_GPIO_ICU)
  device_irq_source_unlink(dev, pv->src, STM32_GPIO_IRQ_SRC_COUNT);
  device_irq_sink_unlink(dev, pv->sink, CONFIG_DRIVER_STM32_GPIO_IRQ_COUNT);
#endif

  mem_free(pv);
}

