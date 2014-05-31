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

#include <device/device.h>
#include <device/driver.h>
#include <device/resources.h>
#include <device/irq.h>
#include <device/class/gpio.h>
#include <device/class/iomux.h>

#include <arch/stm32f4xx_gpio.h>
#include <arch/stm32f4xx_rcc.h>

#include <arch/stm32f4xx_helpers.h>
#include <arch/stm32f4xx_memory_map.h>


#define STM32F4xx_GPIO_BANK_SIZE    16
#define STM32F4xx_GPIO_BANK_COUNT   5
#define STM32F4xx_GPIO_MAX_ID                                \
  (STM32F4xx_GPIO_BANK_SIZE * STM32F4xx_GPIO_BANK_COUNT - 1) \
/**/

struct stm32f4xx_gpio_context_s
{
  /* address of the device. */
  uintptr_t addr;
};


/********************************* GPIO class. **********/

#define STM32_GPIO_MODE_INPUT       1
#define STM32_GPIO_MODE_OUTPUT      2
#define STM32_GPIO_MODE_ANALOG      4
#define STM32_GPIO_MODE_PUSHPULL    8
#define STM32_GPIO_MODE_OPENDRAIN   16
#define STM32_GPIO_MODE_PULLUP      32
#define STM32_GPIO_MODE_PULLDOWN    64

static error_t stm32f4xx_gpio_gpio_make_mode(enum dev_pin_driving_e mode,
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
      *bf = STM32_GPIO_MODE_OUTPUT | STM32_GPIO_MODE_PULLUP;
      break;

    case DEV_PIN_OPENSOURCE_PULLDOWN:
      *bf = STM32_GPIO_MODE_OUTPUT | STM32_GPIO_MODE_PULLDOWN;
      break;
    }
  return 0;
}

static void stm32f4xx_gpio_gpio_apply_mode(struct device_s *dev,
                                           gpio_id_t       iopin,
                                           uint8_t         bf)
{
  struct stm32f4xx_gpio_context_s   *pv  = dev->drv_pv;

  uint8_t bank              = iopin / STM32F4xx_GPIO_BANK_SIZE;
  uint8_t io_in_bank        = iopin % STM32F4xx_GPIO_BANK_SIZE;
  uintptr_t const bkaddr    = pv->addr + (bank * 0x28);

  /* Input/output. */
  if (bf & STM32_GPIO_MODE_OUTPUT)
    STM32F4xx_REG_FIELD_IDX_UPDATE_DEV(
      GPIO,
      bkaddr,
      MODER,
      MODE,
      io_in_bank,
      OUTPUT
    );
  else if (bf & STM32_GPIO_MODE_INPUT)
    STM32F4xx_REG_FIELD_IDX_UPDATE_DEV(
      GPIO,
      bkaddr,
      MODER,
      MODE,
      io_in_bank,
      INPUT
    );

  /* Pushpull/open-drain. */
  if (bf & STM32_GPIO_MODE_PUSHPULL)
    STM32F4xx_REG_FIELD_IDX_UPDATE_DEV(
      GPIO,
      bkaddr,
      OTYPER,
      OT,
      io_in_bank,
      PUSHPULL
    );
  else if (bf & STM32_GPIO_MODE_OPENDRAIN)
    STM32F4xx_REG_FIELD_IDX_UPDATE_DEV(
      GPIO,
      bkaddr,
      OTYPER,
      OT,
      io_in_bank,
      OPEN
    );

  /* Pull-up/pull-down. */
  if (bf & STM32_GPIO_MODE_PULLUP)
    STM32F4xx_REG_FIELD_IDX_UPDATE_DEV(
      GPIO,
      bkaddr,
      PUPDR,
      PUPD,
      io_in_bank,
      PULLUP
    );
  else if (bf & STM32_GPIO_MODE_PULLDOWN)
    STM32F4xx_REG_FIELD_IDX_UPDATE_DEV(
      GPIO,
      bkaddr,
      PUPDR,
      PUPD,
      io_in_bank,
      PULLDOWN
    );
}

static DEVGPIO_SET_MODE(stm32f4xx_gpio_gpio_set_mode)
{
  struct device_s   *dev = gpio->dev;

  uint8_t           msk_idx = 0, bf;

  if (io_first > io_last || io_last > STM32F4xx_GPIO_MAX_ID)
    return -ERANGE;

  if (stm32f4xx_gpio_gpio_make_mode(mode, &bf))
    return -ENOTSUP;

  LOCK_SPIN_IRQ(&dev->lock);

  for (; io_first <= io_last && msk_idx < 64; ++io_first)
    {
      if ((mask[msk_idx/8] >> (msk_idx % 8)) & 0x1)
        {
          stm32f4xx_gpio_gpio_apply_mode(dev, io_first, bf);
        }
      ++msk_idx;
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}

static DEVGPIO_SET_OUTPUT(stm32f4xx_gpio_gpio_set_output)
{
  struct device_s                   *dev = gpio->dev;
  struct stm32f4xx_gpio_context_s   *pv  = dev->drv_pv;

  uint8_t                           msk_idx = 0;

  if (io_first > io_last || io_last > STM32F4xx_GPIO_MAX_ID)
    return -ERANGE;

  LOCK_SPIN_IRQ(&dev->lock);

  for (; io_first <= io_last && msk_idx < 64; ++io_first)
    {
      uint8_t bank              = io_first / STM32F4xx_GPIO_BANK_SIZE;
      uint8_t io_in_bank        = io_first % STM32F4xx_GPIO_BANK_SIZE;
      uintptr_t const bkaddr    = pv->addr + (bank * 0x28);

      uint8_t sval = (set_mask[msk_idx/8] >> (msk_idx % 8)) & 0x1;
      uint8_t cval = (clear_mask[msk_idx/8] >> (msk_idx % 8)) & 0x1;

      ++msk_idx;

      /* sval cval concatenated. */
      switch ((sval << 1) | cval)
      {
      default: break;

      /* clear */
      case 0 /* 00 */:
        STM32F4xx_REG_FIELD_IDX_SET_DEV(GPIO, bkaddr, BSRR, BR, io_in_bank);
        break;

      /* unchanged */
      case 1 /* 01 */:
        break;

      /* toggle */
      case 2 /* 10 */: {
        uint32_t register odrval = STM32F4xx_REG_FIELD_IDX_VALUE_DEV(
          GPIO,
          bkaddr,
          ODR,
          OD,
          io_in_bank
        );
        STM32F4xx_REG_FIELD_IDX_UPDATE_DEV(
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
        STM32F4xx_REG_FIELD_IDX_SET_DEV(GPIO, bkaddr, BSRR, BS, io_in_bank);
        break;
      }
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}

static DEVGPIO_GET_INPUT(stm32f4xx_gpio_gpio_get_input)
{
  struct device_s                   *dev = gpio->dev;
  struct stm32f4xx_gpio_context_s   *pv  = dev->drv_pv;

  if (io_first > io_last || io_last > STM32F4xx_GPIO_MAX_ID)
    return -ERANGE;

  LOCK_SPIN_IRQ(&dev->lock);

  for (; io_first <= io_last; io_first += 32)
    {
      uint8_t bank              = io_first / STM32F4xx_GPIO_BANK_SIZE;
      uintptr_t const bkaddr    = pv->addr + (bank * 0x28);

      uint32_t register value = STM32F4xx_REG_VALUE_DEV(GPIO, bkaddr, IDR);
      *(uint32_t*)data = value >> io_first;

      data += 4;
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}


static const struct driver_gpio_s stm32f4xx_gpio_gpio_drv =
{
  .class_       = DRIVER_CLASS_GPIO,
  .f_set_mode   = &stm32f4xx_gpio_gpio_set_mode,
  .f_set_output = &stm32f4xx_gpio_gpio_set_output,
  .f_get_input  = &stm32f4xx_gpio_gpio_get_input,
  .f_watch      = (devgpio_watch_t*)&dev_driver_notsup_fcn,
  .f_cancel     = (devgpio_cancel_t*)&dev_driver_notsup_fcn,
};


/********************************* IOMUX class. **********/

static DEVIOMUX_SETUP(stm32f4xx_gpio_iomux_setup)
{
  struct device_s                   *dev = imdev->dev;
  struct stm32f4xx_gpio_context_s   *pv  = dev->drv_pv;

  uint8_t                           bf;

  uint8_t bank              = io_id / STM32F4xx_GPIO_BANK_SIZE;
  uint8_t io_in_bank        = io_id % STM32F4xx_GPIO_BANK_SIZE;
  uintptr_t const bkaddr    = pv->addr + (bank * 0x28);

  if (io_id > STM32F4xx_GPIO_MAX_ID)
    return -ERANGE;

  if (stm32f4xx_gpio_gpio_make_mode(dir, &bf))
    return -ENOTSUP;

  /* alternate functions between 0 and 15. */
  if (mux > 15)
    return -EINVAL;

  /* configure the alternate function (number in mux argument). */

  STM32F4xx_REG_FIELD_IDX_UPDATE_DEV(
    GPIO,
    bkaddr,
    MODER,
    MODE,
    io_in_bank,
    ALT
  );

  STM32F4xx_REG_FIELD_IDX_UPDATE_DEV(
    GPIO,
    bkaddr,
    AFRL,
    AF,
    (io_in_bank % 8),
    mux
  );

  /* remove input/output in bitfield as we use here alternate function. */
  bf &= ~(STM32_GPIO_MODE_INPUT | STM32_GPIO_MODE_OUTPUT);
  stm32f4xx_gpio_gpio_apply_mode(dev, io_id, bf);

  return 0;
}


static const struct driver_iomux_s stm32f4xx_gpio_iomux_drv =
{
  .class_   = DRIVER_CLASS_IOMUX,
  .f_setup  = &stm32f4xx_gpio_iomux_setup,
};


/********************************* DRIVER */

static DEV_INIT(stm32f4xx_gpio_init);
static DEV_CLEANUP(stm32f4xx_gpio_cleanup);

struct driver_s stm32f4xx_gpio_drv =
  {
    .desc       = "STM32F4xx GPIO",
    .f_init     = &stm32f4xx_gpio_init,
    .f_cleanup  = &stm32f4xx_gpio_cleanup,
    .classes    =
      {
        &stm32f4xx_gpio_gpio_drv,
        &stm32f4xx_gpio_iomux_drv,
      },
  };

static DEV_INIT(stm32f4xx_gpio_init)
{
  struct stm32f4xx_gpio_context_s   *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;

  dev->drv_pv = pv;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

  dev->drv = &stm32f4xx_gpio_drv;

  /* enable clock gating for gpio A..E. */
  STM32F4xx_REG_UPDATE(RCC, , AHB1ENR, 0x1f);

  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;

err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(stm32f4xx_gpio_cleanup)
{
  struct stm32f4xx_gpio_context_s   *pv = dev->drv_pv;

  /* disable clock gating for gpio A..E. */
  STM32F4xx_REG_UPDATE(RCC, , AHB1ENR, 0x0);

  mem_free(pv);
}

