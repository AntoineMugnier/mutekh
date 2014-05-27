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

    Copyright Julien Peeters <contact@julienpeeters.net> (c) 2014

*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <hexo/interrupt.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/resources.h>
#include <device/device.h>
#include <device/driver.h>
#include <device/irq.h>
#include <device/class/i2c.h>

#include <arch/stm32f4xx_regs.h>

struct stm32f4xx_i2c_context_s
{
  uintptr_t addr;
};


/***************************************** config */

DEVI2C_CTRL_CONFIG(stm32f4xx_i2c_config)
{
  return -ENOTSUP;
}


/***************************************** transfer */

static void stm32f4xx_i2c_start(const struct device_i2c_ctrl_s *i2cdev,
                                struct dev_i2c_ctrl_transfer_s *tr)
{
  struct stm32f4xx_i2c_context_s    *pv = i2cdev->dev->drv_pv;

  /* 10 bits addressing not supported by the driver. */
  if (tr->amode == DEV_I2C_ADDR_10_BITS)
    {
      tr->error = ENOTSUP;
      return;
    }

  /* see p. 466 of Reference manual. */

  /* 0. wait if the bus is busy. */
  if (STM32F4xx_REG_FIELD_VALUE_DEV(I2C, pv->addr, SR2, BUSY) != 0)
    {
      tr->error = EBUSY;
      return;
    }

  /* 1. generate the start condition and wait for the start condition to be
   * sent. This force the master mode.
   */
  STM32F4xx_REG_FIELD_SET_DEV(I2C, pv->addr, CR1, START);
  while (STM32F4xx_REG_FIELD_VALUE_DEV(I2C, pv->addr, SR1, SB) == 0);

  /* 2. send the address. */
  STM32F4xx_REG_UPDATE_DEV(
    I2C,
    pv->addr,
    DR,
    (tr->saddr << 1) | (tr->dir == DEV_I2C_TR_READ ? 0x1 : 0x0)
  );

  /* 3. wait for the acknowledge. */
  while(STM32F4xx_REG_FIELD_VALUE_DEV(I2C, pv->addr, SR1, ADDRESS) == 0)
  {
    uint32_t sr1 = STM32F4xx_REG_VALUE_DEV(I2C, pv->addr, SR1);

    if ((sr1 & (STM32F4xx_I2C_SR1_TIMEOUT | STM32F4xx_I2C_SR1_AF)) != 0)
    {
      STM32F4xx_REG_FIELD_CLR_DEV(I2C, pv->addr, SR1, TIMEOUT);
      STM32F4xx_REG_FIELD_CLR_DEV(I2C, pv->addr, SR1, AF);
      STM32F4xx_REG_FIELD_SET_DEV(I2C, pv->addr, CR1, STOP);

      if (sr1 & STM32F4xx_I2C_SR1_AF)
        tr->error = EADDRNOTAVAIL;

      if (sr1 & STM32F4xx_I2C_SR1_TIMEOUT)
        tr->error = ETIMEDOUT;

      kroutine_exec(&tr->kr, cpu_is_interruptible());
      return;
    }
  }

  /* 4. clear the acknowledge. */
  (void) STM32F4xx_REG_VALUE_DEV(I2C, pv->addr, SR2);
}

static void stm32f4xx_i2c_stop (const struct device_i2c_ctrl_s *i2cdev)
{
  struct stm32f4xx_i2c_context_s    *pv = i2cdev->dev->drv_pv;

  STM32F4xx_REG_FIELD_SET_DEV(I2C, pv->addr, CR1, STOP);
}

DEVI2C_CTRL_TRANSFER(stm32f4xx_i2c_transfer)
{
  switch (tr->dir)
    {
    case DEV_I2C_TR_WRITE:
      /* scan. */
      if (tr->count == 0)
        {
          stm32f4xx_i2c_start(i2cdev, tr);
          stm32f4xx_i2c_stop(i2cdev);
          break;
        }

      /* write. */

    case DEV_I2C_TR_READ:
      /* read. */

    default:
      tr->error = ENOTSUP;
      break;
    }
}

static const struct driver_i2c_ctrl_s stm32f4xx_i2c_drv_cls =
{
  .class_       = DRIVER_CLASS_I2C,
  .f_config     = &stm32f4xx_i2c_config,
  .f_transfer   = &stm32f4xx_i2c_transfer,
};

static DEV_INIT(stm32f4xx_i2c_init);
static DEV_CLEANUP(stm32f4xx_i2c_cleanup);

const struct driver_s stm32f4xx_i2c_drv =
{
  .desc         = "STMF4xx I2C",
  .f_init       = &stm32f4xx_i2c_init,
  .f_cleanup    = &stm32f4xx_i2c_cleanup,
  .classes      =
  {
    &stm32f4xx_i2c_drv_cls,
    0
  }
};

REGISTER_DRIVER(stm32f4xx_i2c_drv);

static DEV_INIT(stm32f4xx_i2c_init)
{
  struct stm32f4xx_i2c_context_s    *pv;
  uint16_t                          freq_in_mhz;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  /* allocate driver private context. */
  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  if (!pv)
  {
    return -ENOMEM;
  }
  dev->drv_pv = pv;

  /* retreive the device base address from device tree. */
  if(device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
  {
    goto err_mem;
  }

  /* reset the device. */
  STM32F4xx_REG_UPDATE_DEV(I2C, pv->addr, CR1, 0);
  STM32F4xx_REG_UPDATE_DEV(I2C, pv->addr, CR2, 0);

  /* configure GPIO. */
  // XXX: will be fixed by pinmux. */
  switch (pv->addr)
  {
  default: assert(0 && "unknown I2C controller");

  case STM32F4xx_I2C1_ADDR:
  {
    /* enable GPIO B. */
    STM32F4xx_REG_FIELD_SET(RCC, , AHB1ENR, GPIOBEN);

    /* set pin mode to alternate functions for PB8/PB9. */
    uint32_t register cfg = STM32F4xx_REG_VALUE(GPIO, B, MODER);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, MODER, MODE, 8, ALT, cfg);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, MODER, MODE, 9, ALT, cfg);
    STM32F4xx_REG_UPDATE(GPIO, B, MODER, cfg);

    /* set alternate function to 4 (I2C) on pins PB8/PB9. */
    cfg = STM32F4xx_REG_VALUE(GPIO, B, AFRH);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, AFRH, AF, 0, 4, cfg);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, AFRH, AF, 1, 4, cfg);
    STM32F4xx_REG_UPDATE(GPIO, B, AFRH, cfg);

    /* set pins PB8/PB9 as open-drain as required by the I2C bu
     * (we are the master).
     */
    cfg = STM32F4xx_REG_VALUE(GPIO, B, OTYPER);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, OTYPER, OT, 8, OPEN, cfg);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, OTYPER, OT, 9, OPEN, cfg);
    STM32F4xx_REG_UPDATE(GPIO, B, OTYPER, cfg);

    /* set pins PB8/PB9 as pull-up as required by the I2C bu
     * (we are the master).
     */
    cfg = STM32F4xx_REG_VALUE(GPIO, B, PUPDR);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, PUPDR, PUPD, 8, PULLUP, cfg);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, PUPDR, PUPD, 9, PULLUP, cfg);
    STM32F4xx_REG_UPDATE(GPIO, B, PUPDR, cfg);

    break;
  }

  case STM32F4xx_I2C2_ADDR:
  {
    /* enable GPIO B. */
    STM32F4xx_REG_FIELD_SET(RCC, , AHB1ENR, GPIOBEN);

    /* set pin mode to alternate functions for pins PB10/PB3. */
    uint32_t register cfg = STM32F4xx_REG_VALUE(GPIO, B, MODER);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, MODER, MODE, 10, ALT, cfg);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, MODER, MODE, 3, ALT, cfg);
    STM32F4xx_REG_UPDATE(GPIO, B, MODER, cfg);

    /* set alternate function to 4 (I2C) on SCL on PB10. */
    STM32F4xx_REG_FIELD_IDX_UPDATE(GPIO, B, AFRH, AF, 2, 4);

    /* set alternate function to 9 (I2C) on SDA on PB3. */
    STM32F4xx_REG_FIELD_IDX_UPDATE(GPIO, B, AFRL, AF, 3, 9);

    /* set pins PB10/PB3 as open-drain as required by the I2C bu
     * (we are the master).
     */
    cfg = STM32F4xx_REG_VALUE(GPIO, B, OTYPER);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, OTYPER, OT, 10, OPEN, cfg);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, OTYPER, OT, 3, OPEN, cfg);
    STM32F4xx_REG_UPDATE(GPIO, B, OTYPER, cfg);

    /* set pin PB10/PB3 as pull-up as required by the I2C bus
     * (we are the master). */
    cfg = STM32F4xx_REG_VALUE(GPIO, B, PUPDR);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, PUPDR, PUPD, 10, PULLUP, cfg);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, PUPDR, PUPD, 3, PULLUP, cfg);
    STM32F4xx_REG_UPDATE(GPIO, B, PUPDR, cfg);

    break;
  }

  case STM32F4xx_I2C3_ADDR:
  {
    /* enable GPIO A. */
    STM32F4xx_REG_FIELD_SET(RCC, , AHB1ENR, GPIOAEN);

    /* set pin mode to alternate functions for pins PA8/PA4. */
    uint32_t register cfg = STM32F4xx_REG_VALUE(GPIO, A, MODER);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, MODER, MODE, 8, ALT, cfg);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, MODER, MODE, 4, ALT, cfg);
    STM32F4xx_REG_UPDATE(GPIO, A, MODER, cfg);

    /* set alternate function to 4 (I2C) on PA8. */
    STM32F4xx_REG_FIELD_IDX_UPDATE(GPIO, A, AFRH, AF, 0 /* 8 */, 4);

    /* set alternate function to 9 (I2C) on PA4 . */
    STM32F4xx_REG_FIELD_IDX_UPDATE(GPIO, A, AFRL, AF, 4, 9);

    /* set pins PA8/PA4 as open-drain as required by the I2C bu
     * (we are the master).
     */
    cfg = STM32F4xx_REG_VALUE(GPIO, A, OTYPER);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, OTYPER, OT, 8, OPEN, cfg);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, OTYPER, OT, 4, OPEN, cfg);
    STM32F4xx_REG_UPDATE(GPIO, A, OTYPER, cfg);

    /* set pin PA8/PA4 as pull-up as required by the I2C bus
     * (we are the master). */
    cfg = STM32F4xx_REG_VALUE(GPIO, A, PUPDR);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, PUPDR, PUPD, 8, PULLUP, cfg);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, PUPDR, PUPD, 4, PULLUP, cfg);
    STM32F4xx_REG_UPDATE(GPIO, A, PUPDR, cfg);

    break;
  }
  }

  /* configure input clock. */
  // XXX: will be fixed by clock tree. */
  switch (pv->addr)
  {
  default: assert(0 && "unknown I2C controller");

  case STM32F4xx_I2C1_ADDR:
    STM32F4xx_REG_FIELD_SET(RCC, , APB1ENR, I2C1EN);
    break;

  case STM32F4xx_I2C2_ADDR:
    STM32F4xx_REG_FIELD_SET(RCC, , APB1ENR, I2C2EN);
    break;

  case STM32F4xx_I2C3_ADDR:
    STM32F4xx_REG_FIELD_SET(RCC, , APB1ENR, I2C3EN);
    break;
  }

  /* initialize the input clock frequency. */
  extern uint32_t stm32f4xx_clock_freq_apb1;

  freq_in_mhz = stm32f4xx_clock_freq_apb1 / 1000000;
  STM32F4xx_REG_FIELD_UPDATE_DEV(I2C, pv->addr, CR2, FREQ, freq_in_mhz);

  /* set standard mode. */
  STM32F4xx_REG_FIELD_UPDATE_DEV(I2C, pv->addr, CCR, FS, SM);

  /* initialize the I2C bus speed (100kHz in standard mode). */
  STM32F4xx_REG_FIELD_UPDATE_DEV(I2C, pv->addr, CCR, CCR, 5 * freq_in_mhz);

  /* initialize the rise time (p. 491 of Reference manual). */
  STM32F4xx_REG_UPDATE_DEV(I2C, pv->addr, TRISE, freq_in_mhz + 1);

  /* enable I2C device. */
  STM32F4xx_REG_FIELD_SET_DEV(I2C, pv->addr, CR1, PE);

  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;

err_mem:
  mem_free(pv);
  return -EINVAL;
}

static DEV_CLEANUP(stm32f4xx_i2c_cleanup)
{
  struct stm32f4xx_i2c_context_s    *pv;

  pv = dev->drv_pv;

  // XXX: Wait for the current transmission.

  /* disable I2C device. */
  STM32F4xx_REG_UPDATE_DEV(I2C, pv->addr, CR1, 0);
  STM32F4xx_REG_UPDATE_DEV(I2C, pv->addr, CR2, 0);
  STM32F4xx_REG_UPDATE_DEV(I2C, pv->addr, CCR, 0);

  /* de-configure gpio. */
  switch (pv->addr)
  {
  default: assert(0 && "unknown I2C controller");

  case STM32F4xx_I2C1_ADDR:
  {
    /* reset pin PB8/PB9. */
    uint32_t register cfg = STM32F4xx_REG_VALUE(GPIO, B, MODER);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, MODER, MODE, 8, INPUT, cfg);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, MODER, MODE, 9, INPUT, cfg);
    STM32F4xx_REG_UPDATE(GPIO, B, MODER, cfg);

    cfg = STM32F4xx_REG_VALUE(GPIO, B, PUPDR);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, PUPDR, PUPD, 6, NONE, cfg);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, PUPDR, PUPD, 7, NONE, cfg);
    STM32F4xx_REG_UPDATE(GPIO, B, PUPDR, cfg);

    break;
  }

  case STM32F4xx_I2C2_ADDR:
  {
    /* reset pins PB10/PB3. */
    uint32_t register cfg = STM32F4xx_REG_VALUE(GPIO, B, MODER);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, MODER, MODE, 10, INPUT, cfg);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, MODER, MODE, 3, INPUT, cfg);
    STM32F4xx_REG_UPDATE(GPIO, B, MODER, cfg);

    cfg = STM32F4xx_REG_VALUE(GPIO, B, PUPDR);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, PUPDR, PUPD, 10, PULLUP, cfg);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, PUPDR, PUPD, 3, PULLUP, cfg);
    STM32F4xx_REG_UPDATE(GPIO, B, PUPDR, cfg);

    break;
  }

  case STM32F4xx_I2C3_ADDR:
  {
    /* reset pins PA8/PA4. */
    uint32_t register cfg = STM32F4xx_REG_VALUE(GPIO, A, MODER);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, MODER, MODE, 8, INPUT, cfg);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, MODER, MODE, 4, INPUT, cfg);
    STM32F4xx_REG_UPDATE(GPIO, A, MODER, cfg);

    cfg = STM32F4xx_REG_VALUE(GPIO, A, PUPDR);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, PUPDR, PUPD, 8, PULLUP, cfg);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, PUPDR, PUPD, 4, PULLUP, cfg);
    STM32F4xx_REG_UPDATE(GPIO, A, PUPDR, cfg);
    break;
  }
  }

  /* configure input clock. */
  // XXX: will be fixed by clock tree. */
  switch (pv->addr)
  {
  default: assert(0 && "unknown I2C controller");

  case STM32F4xx_I2C1_ADDR:
    STM32F4xx_REG_FIELD_CLR(RCC, , APB1ENR, I2C1EN);
    break;

  case STM32F4xx_I2C2_ADDR:
    STM32F4xx_REG_FIELD_CLR(RCC, , APB1ENR, I2C2EN);
    break;

  case STM32F4xx_I2C3_ADDR:
    STM32F4xx_REG_FIELD_CLR(RCC, , APB1ENR, I2C3EN);
    break;
  }

  /* deallocate private driver context. */
  mem_free(pv);
}

