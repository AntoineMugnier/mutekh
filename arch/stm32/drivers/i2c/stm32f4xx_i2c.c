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

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/irq.h>
#include <device/class/i2c.h>

#include <arch/stm32f4xx_regs.h>

struct stm32f4xx_i2c_context_s
{
  uintptr_t            addr;
  dev_i2c_queue_root_t queue;
};

DEVI2C_REQUEST(stm32f4xx_i2c_request)
{

}

DEVI2C_SET_BAUDRATE(stm32f4xx_i2c_set_baudrate)
{
  return 0;
}

static const struct driver_i2c_s stm32f4xx_i2c_drv_cls =
{
  .cl               = DRIVER_CLASS_I2C,
  .f_request        = &stm32f4xx_i2c_request,
  .f_set_baudrate   = &stm32f4xx_i2c_set_baudrate
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
    /* set pin mode to alternate functions. */
    uint32_t register cfg = STM32F4xx_REG_VALUE(GPIO, B, MODER);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, MODER, MODE, 6, ALT, cfg);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, MODER, MODE, 7, ALT, cfg);
    STM32F4xx_REG_UPDATE(GPIO, B, MODER, cfg);

    /* set alternate function to 4 (I2C) on pins. */
    cfg = STM32F4xx_REG_VALUE(GPIO, B, AFRL);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, AFRL, AF, 6, 4, cfg);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, AFRL, AF, 7, 4, cfg);
    STM32F4xx_REG_UPDATE(GPIO, B, AFRL, cfg);

    /* set pin as pull-up as required by the I2C bus (we are the master). */
    cfg = STM32F4xx_REG_VALUE(GPIO, B, PUPDR);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, PUPDR, PUPD, 6, PULLUP, cfg);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, PUPDR, PUPD, 7, PULLUP, cfg);
    STM32F4xx_REG_UPDATE(GPIO, B, PUPDR, cfg);
    break;
  }

  case STM32F4xx_I2C2_ADDR:
  {
    /* set pin mode to alternate functions. */
    uint32_t register cfg = STM32F4xx_REG_VALUE(GPIO, B, MODER);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, MODER, MODE, 10, ALT, cfg);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, MODER, MODE, 3, ALT, cfg);
    STM32F4xx_REG_UPDATE(GPIO, B, MODER, cfg);

    /* set alternate function to 4 (I2C) on SCL. */
    STM32F4xx_REG_FIELD_IDX_UPDATE(GPIO, B, AFRH, AF, 2 /* 10 */, 4);

    /* set alternate function to 9 (I2C) on SDA. */
    STM32F4xx_REG_FIELD_IDX_UPDATE(GPIO, B, AFRL, AF, 3, 9);

    /* set pin as pull-up as required by the I2C bus (we are the master). */
    cfg = STM32F4xx_REG_VALUE(GPIO, B, PUPDR);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, PUPDR, PUPD, 6, PULLUP, cfg);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, PUPDR, PUPD, 7, PULLUP, cfg);
    STM32F4xx_REG_UPDATE(GPIO, B, PUPDR, cfg);
    break;
  }

  case STM32F4xx_I2C3_ADDR:
  {
    /* set pin mode to alternate functions. */
    uint32_t register cfg = STM32F4xx_REG_VALUE(GPIO, A, MODER);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, MODER, MODE, 8, ALT, cfg);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, MODER, MODE, 4, ALT, cfg);
    STM32F4xx_REG_UPDATE(GPIO, A, MODER, cfg);

    /* set alternate function to 4 (I2C) on pins. */
    STM32F4xx_REG_FIELD_IDX_UPDATE(GPIO, B, AFRH, AF, 0 /* 8 */, 4);

    /* set alternate function to 9 (I2C) on SDA. */
    STM32F4xx_REG_FIELD_IDX_UPDATE(GPIO, B, AFRL, AF, 4, 9);

    /* set pin as pull-up as required by the I2C bus (we are the master). */
    cfg = STM32F4xx_REG_VALUE(GPIO, B, PUPDR);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, PUPDR, PUPD, 6, PULLUP, cfg);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, PUPDR, PUPD, 7, PULLUP, cfg);
    STM32F4xx_REG_UPDATE(GPIO, B, PUPDR, cfg);
    break;
  }
  }

  /* configure input clock. */
  // XXX: will be fixed by clock tree. */
  switch (pv->addr)
  {
  default: assert(0 && "unknown I2C controller");

#define EMPTY()

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
  STM32F4xx_REG_FIELD_UPDATE_DEV(I2C, pv->addr, CR2, FREQ, 5 * freq_in_mhz);

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
    uint32_t register cfg = STM32F4xx_REG_VALUE(GPIO, B, MODER);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, MODER, MODE, 6, INPUT, cfg);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, MODER, MODE, 7, INPUT, cfg);
    STM32F4xx_REG_UPDATE(GPIO, B, MODER, cfg);
    break;
  }

  case STM32F4xx_I2C2_ADDR:
  {
    uint32_t register cfg = STM32F4xx_REG_VALUE(GPIO, B, MODER);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, MODER, MODE, 10, INPUT, cfg);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, MODER, MODE, 3, INPUT, cfg);
    STM32F4xx_REG_UPDATE(GPIO, B, MODER, cfg);
    break;
  }

  case STM32F4xx_I2C3_ADDR:
  {
    uint32_t register cfg = STM32F4xx_REG_VALUE(GPIO, A, MODER);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, MODER, MODE, 8, INPUT, cfg);
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, MODER, MODE, 4, INPUT, cfg);
    STM32F4xx_REG_UPDATE(GPIO, A, MODER, cfg);
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

