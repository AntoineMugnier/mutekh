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

    Copyright (c) 2015 Julien Peeters <contact@julienpeeters.net>

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

#include <arch/stm32/exti.h>
#include <arch/stm32/syscfg.h>

#include <arch/stm32/mmap.h>
#include <arch/stm32/gpio.h>


#define STM32_GPIO_BANK_SIZE        16
#define STM32_GPIO_BANK_COUNT       7
#define STM32_GPIO_BANK_MASK        ((1 << STM32_GPIO_BANK_SIZE)-1)
#define STM32_GPIO_MAX_ID           (STM32_GPIO_BANK_SIZE * STM32_GPIO_BANK_COUNT - 1)
#define STM32_GPIO_IRQ_SRC_COUNT    7

DRIVER_PV(struct stm32_gpio_private_s
{
#if defined(CONFIG_DRIVER_STM32_GPIO_X4_ICU)
  /* This specifies which bank is selected for each interrupt line. A
     value of -1 means that no bank is currently bound to an
     interrupt. */
  struct {
      int8_t bank:3;
  }                     irq[STM32_GPIO_BANK_SIZE];

  struct dev_irq_sink_s sink[STM32_GPIO_BANK_SIZE];
  struct dev_irq_src_s  src[STM32_GPIO_IRQ_SRC_COUNT];
#endif
});


/********************************* GPIO class. **********/

#define STM32_GPIO_REG_MODE     0
#define STM32_GPIO_REG_OTYPE    1
#define STM32_GPIO_REG_PUPD     2

static
error_t stm32_gpio_prepare_mode(enum dev_pin_driving_e mode,
                                uint8_t                mask[3])
{
  *mask = 0;

  switch (mode)
    {
    default:
      return -ENOTSUP;

    case DEV_PIN_DISABLED:
      break;

    case DEV_PIN_PUSHPULL:
      mask[STM32_GPIO_REG_MODE]  = STM32_GPIO_MODER_MODE_OUTPUT;
      mask[STM32_GPIO_REG_OTYPE] = STM32_GPIO_OTYPER_OT_PUSHPULL;
      mask[STM32_GPIO_REG_PUPD]  = 0;
      break;

    case DEV_PIN_INPUT:
      mask[STM32_GPIO_REG_MODE]  = STM32_GPIO_MODER_MODE_INPUT;
      mask[STM32_GPIO_REG_OTYPE] = 0;
      mask[STM32_GPIO_REG_PUPD]  = 0;
      break;

    case DEV_PIN_INPUT_PULLUP:
      mask[STM32_GPIO_REG_MODE]  = STM32_GPIO_MODER_MODE_INPUT;
      mask[STM32_GPIO_REG_OTYPE] = 0;
      mask[STM32_GPIO_REG_PUPD]  = STM32_GPIO_PUPDR_PUPD_PULLUP;
      break;

    case DEV_PIN_INPUT_PULLDOWN:
      mask[STM32_GPIO_REG_MODE]  = STM32_GPIO_MODER_MODE_INPUT;
      mask[STM32_GPIO_REG_OTYPE] = 0;
      mask[STM32_GPIO_REG_PUPD]  = STM32_GPIO_PUPDR_PUPD_PULLDOWN;
      break;

    case DEV_PIN_OPENDRAIN:
      mask[STM32_GPIO_REG_MODE]  = STM32_GPIO_MODER_MODE_OUTPUT;
      mask[STM32_GPIO_REG_OTYPE] = STM32_GPIO_OTYPER_OT_OPENDRAIN;
      mask[STM32_GPIO_REG_PUPD]  = 0;
      break;

    case DEV_PIN_OPENSOURCE:
      mask[STM32_GPIO_REG_MODE]  = STM32_GPIO_MODER_MODE_OUTPUT;
      mask[STM32_GPIO_REG_OTYPE] = STM32_GPIO_OTYPER_OT_OPENDRAIN;
      mask[STM32_GPIO_REG_PUPD]  = 0;
      break;

    case DEV_PIN_OPENDRAIN_PULLUP:
      mask[STM32_GPIO_REG_MODE]  = STM32_GPIO_MODER_MODE_OUTPUT;
      mask[STM32_GPIO_REG_OTYPE] = STM32_GPIO_OTYPER_OT_OPENDRAIN;
      mask[STM32_GPIO_REG_PUPD]  = STM32_GPIO_PUPDR_PUPD_PULLUP;
      break;

    case DEV_PIN_OPENSOURCE_PULLDOWN:
      mask[STM32_GPIO_REG_MODE]  = STM32_GPIO_MODER_MODE_OUTPUT;
      mask[STM32_GPIO_REG_OTYPE] = STM32_GPIO_OTYPER_OT_OPENDRAIN;
      mask[STM32_GPIO_REG_PUPD]  = STM32_GPIO_PUPDR_PUPD_PULLDOWN;
      break;
    }

  return 0;
}

static
inline
uint64_t stm32_gpio_get_1bit_mask(uint32_t smask)
{
  return smask;
}

static
inline
uint64_t stm32_gpio_get_2bit_mask(uint32_t smask)
{
  uint32_t p0 = (0x00002412 * (smask & 0x1209)) & 0x02080082;
  uint32_t p1 = (0x000050a8 * (smask & 0x2854)) & 0x08802220;
  uint32_t p2 = (0x00008904 * (smask & 0x4482)) & 0x20208008;
  uint32_t p3 = (0x00010240 * (smask & 0x8120)) & 0x80020800;

  uint64_t r = p0 | p1 | p2 | p3;

  r |= r >> 1;

  return r;
}

static
void stm32_gpio_set_mode_reg(gpio_id_t io_first, gpio_id_t io_last,
                             const uint8_t *mask, uint32_t mode,
                             uint_fast8_t reg)
{
  uint16_t tmask;
  uint32_t smask;
  uint64_t pmask = 0;

  /* GPIO mode is configured in only one register for the 16 pins per bank. */
  uint_fast8_t mshift = io_first % STM32_GPIO_BANK_SIZE;
  int_fast8_t mlen    = io_last - io_first + 1;

mask:
  /* do we need to compute the entire mask byte ? */
  if (mlen > (STM32_GPIO_BANK_SIZE / 2))
    tmask = mlen > STM32_GPIO_BANK_SIZE ? STM32_GPIO_BANK_MASK : ((1 << mlen) - 1);
  else
    tmask = (1 << mlen) - 1;

loop:
  /* do we need to compute the entire mask byte ? */
  if (mlen > (STM32_GPIO_BANK_SIZE / 2))
    {
      smask  = endian_le16_na_load(mask) & tmask;
      mask  += 2;
    }
  else
    smask = *mask++ & tmask;

  /* we compute the relevant bits in the mask and align to the first io. */
  switch (reg)
    {
    case STM32_GPIO_REG_MODE:
    case STM32_GPIO_REG_PUPD:
      /* 2-bit stride */
      pmask |= stm32_gpio_get_2bit_mask(smask) << (mshift * 2);
      break;

    case STM32_GPIO_REG_OTYPE:
      pmask |= stm32_gpio_get_1bit_mask(smask) << mshift;
      break;
    }

  mlen -= STM32_GPIO_BANK_SIZE;

update:;
  uint_fast8_t bank = io_first / STM32_GPIO_BANK_SIZE;
  uintptr_t a       = STM32_GPIO_ADDR;

  switch (reg)
    {
    case STM32_GPIO_REG_MODE:
      a += STM32_GPIO_MODER_ADDR(bank);
      break;

    case STM32_GPIO_REG_OTYPE:
      a += STM32_GPIO_OTYPER_ADDR(bank);
      break;

    case STM32_GPIO_REG_PUPD:
      a += STM32_GPIO_PUPDR_ADDR(bank);
      break;
    }

  uint32_t x = endian_le32(cpu_mem_read_32(a));
  x = (x & ~pmask) | (pmask & mode);
  //printk("update mode reg:%u bank:%u mode:0x%08x\n", reg, bank, x);
  cpu_mem_write_32(a, endian_le32(x));

  /* we compute the relevant bits in the mask and align to the first io. */
  switch (reg)
    {
    case STM32_GPIO_REG_MODE:
    case STM32_GPIO_REG_PUPD:
      /* 2-bit stride */
      pmask >>= STM32_GPIO_BANK_SIZE * 2;
      break;

    case STM32_GPIO_REG_OTYPE:
      pmask >>= STM32_GPIO_BANK_SIZE;
      break;
    }

  io_first = (io_first | (STM32_GPIO_BANK_SIZE - 1)) + 1;

  if (mlen >= STM32_GPIO_BANK_SIZE)
    goto loop;

  if (io_first <= io_last)
    {
      if (mlen < 0)
        goto update;
      goto mask;
    }
}

static
error_t stm32_gpio_apply_mode(gpio_id_t io_first, gpio_id_t io_last,
                              const uint8_t *mask, enum dev_pin_driving_e mode,
                              bool_t alt)
{
  uint8_t mmask[3] = { 0 };

  error_t err = stm32_gpio_prepare_mode(mode, mmask);
  if (err)
    return err;

  if (alt)
    mmask[STM32_GPIO_REG_MODE] = STM32_GPIO_MODER_MODE_ALT;

  /* Input/output/alternate (32-bit mask with 2-bit stride). */
  stm32_gpio_set_mode_reg(io_first, io_last, mask,
    mmask[STM32_GPIO_REG_MODE] * 0x55555555, STM32_GPIO_REG_MODE);
  /* Push-pull/open-drain (32-bit mask with 1-bit stride). */
  stm32_gpio_set_mode_reg(io_first, io_last, mask,
    mmask[STM32_GPIO_REG_OTYPE] * 0xffff, STM32_GPIO_REG_OTYPE);
  /* Pull-up/pull-down (32-bit mask with 2-bit stride). */
  stm32_gpio_set_mode_reg(io_first, io_last, mask,
    mmask[STM32_GPIO_REG_PUPD] * 0x55555555, STM32_GPIO_REG_PUPD);

  return 0;
}

static
error_t stm32_gpio_apply_alt_func(gpio_id_t io_id, uint8_t mux)
{
  uint_fast8_t const bank  = io_id / STM32_GPIO_BANK_SIZE;
  uint_fast8_t const shift = (io_id % STM32_GPIO_BANK_SIZE) * 4 /* stride */;

  if (io_id > STM32_GPIO_MAX_ID)
    return -ERANGE;

  if (mux > 15)
    return -ERANGE;

  uintptr_t a = STM32_GPIO_ADDR + STM32_GPIO_AFRL_ADDR(bank);
  uint64_t  x = endian_le64_na_load((void*)a);
  x = (x & ~(0xfULL << shift)) | ((uint64_t)mux << shift);
  //printk("alt func id:%u f:%u\n", io_id, mux);
  endian_le64_na_store((void*)a, x);

  return 0;
}

static
DEV_GPIO_SET_MODE(stm32_gpio_gpio_set_mode)
{
  struct device_s *dev = gpio->dev;

  if (io_first > io_last || io_last > STM32_GPIO_MAX_ID)
    return -ERANGE;

  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);
  err = stm32_gpio_apply_mode(io_first, io_last, mask, mode, 0 /* alt */);
  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static
DEV_GPIO_SET_OUTPUT(stm32_gpio_gpio_set_output)
{
  struct device_s *dev = gpio->dev;

  uint16_t tmask;
  uint32_t smask = 0, cmask = 0;

  uint_fast8_t mshift = io_first % STM32_GPIO_BANK_SIZE;
  int_fast8_t  mlen   = io_last - io_first + 1;

  if (io_first > io_last || io_last > STM32_GPIO_MAX_ID)
    return -ERANGE;

  LOCK_SPIN_IRQ(&dev->lock);

mask:
  tmask = mlen > STM32_GPIO_BANK_SIZE ? 0xffff : ((1 << mlen) - 1);

loop:
  smask |= ((uint32_t)endian_le16_na_load(set_mask) & tmask) << mshift;
  cmask |= ((uint32_t)endian_le16_na_load(clear_mask) & tmask) << mshift;
  cmask |= ~(tmask << mshift);

  set_mask   += 2;
  clear_mask += 2;

  mlen -= STM32_GPIO_BANK_SIZE;

update:;
  uint_fast8_t bank = io_first / STM32_GPIO_BANK_SIZE;

  uintptr_t a = STM32_GPIO_ADDR + STM32_GPIO_ODR_ADDR(bank);
  uint32_t  x = endian_le32(cpu_mem_read_32(a)) & 0xffff;
  x = smask ^ (x & (smask ^ cmask));
  //printk("update odr bank:%u reg:0x%x\n", bank, pmask);
  cpu_mem_write_32(a, endian_le32(x));

  smask >>= STM32_GPIO_BANK_SIZE;
  cmask >>= STM32_GPIO_BANK_SIZE;

  io_first = (io_first | (STM32_GPIO_BANK_SIZE - 1)) + 1;

  if (mlen >= STM32_GPIO_BANK_SIZE)
    goto loop;

  if (io_first <= io_last)
    {
      if (mlen < 0)
        goto update;
      goto mask;
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}

static
DEV_GPIO_GET_INPUT(stm32_gpio_gpio_get_input)
{
  struct device_s *dev = gpio->dev;

  uint32_t vp, v;
  uint_fast8_t bf = io_first / STM32_GPIO_BANK_SIZE;
  uint_fast8_t bl = io_last / STM32_GPIO_BANK_SIZE;
  uint_fast8_t shift = io_first % STM32_GPIO_BANK_SIZE;

  if (io_first > io_last || io_last > STM32_GPIO_MAX_ID)
    return -ERANGE;

  LOCK_SPIN_IRQ(&dev->lock);

  vp = endian_le32(
    cpu_mem_read_32(STM32_GPIO_ADDR + STM32_GPIO_IDR_ADDR(bf)));
  vp >>= shift;

  while (bf++ < bl)
    {
      v = endian_le32(
        cpu_mem_read_32(STM32_GPIO_ADDR + STM32_GPIO_IDR_ADDR(bf)));
      v = (v << (STM32_GPIO_BANK_SIZE - shift)) | vp ;
      vp = v >> STM32_GPIO_BANK_SIZE;

      endian_le16_na_store(data, v);
      data += 2;
    }

  endian_le16_na_store(data, vp);

  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}

/********************************* IOMUX class. **********/

static
DEV_IOMUX_SETUP(stm32_gpio_iomux_setup)
{
  struct device_s *dev = accessor->dev;
  error_t         err;

  if (io_id > STM32_GPIO_MAX_ID)
    return -ERANGE;

  LOCK_SPIN_IRQ(&dev->lock);

  /* set pin mode */
  uint8_t mask[1] = { 0x1 };
  err = stm32_gpio_apply_mode(io_id, io_id, mask, dir, 1 /* alt */);
  if (err)
    goto end;

  /* configure the alternate function (number in mux argument). */
  if (mux != IOMUX_INVALID_MUX)
    err = stm32_gpio_apply_alt_func(io_id, mux);

end:
  LOCK_RELEASE_IRQ(&dev->lock);
  return err;
}

/********************************* ICU class. **********/

#if defined(CONFIG_DRIVER_STM32_GPIO_X4_ICU)

static
uint_fast8_t stm32_gpio_icu_src_id_of_sink_id(uint_fast8_t sink_id)
{
  switch (sink_id)
    {
    default:
      assert(0 && "non reachable.");
      return 0;

    case 0 ... 4:
      // No interrupt sharing for EXTI0...4
      return 0;

    case 5 ... 9:
      // Interrupt sharing for EXTI5...9
      return sink_id - 5;

    case 10 ... 15:
      // Interrupt sharing for EXTI10...15
      return sink_id - 10;
    }
}

static
DEV_IRQ_SINK_UPDATE(stm32_gpio_icu_sink_update)
{
  struct device_s             *dev = sink->base.dev;
  struct stm32_gpio_private_s *pv  = dev->drv_pv;

  uint_fast8_t const sink_id = sink - pv->sink;
  uint_fast8_t       f, r;

  uintptr_t a;
  uint32_t  x;

  switch (sense)
    {
    default:
      assert(0 && "unsupported sensing");
      return;

    case DEV_IRQ_SENSE_NONE:
      r = f = 0;
      return;

    case DEV_IRQ_SENSE_FALLING_EDGE:
      r = 0;
      f = 1;
      break;

    case DEV_IRQ_SENSE_RISING_EDGE:
      r = 1;
      f = 0;
      break;

    case DEV_IRQ_SENSE_ANY_EDGE:
      r = 1;
      f = 1;
      break;
    }

  /* Disable interrupt. */
  if (r == 0 && f == 0)
    {
#if CONFIG_STM32_FAMILY == 4
      a = STM32_EXTI_ADDR + STM32_EXTI_IMR_ADDR;
      x = endian_le32(cpu_mem_read_32(a));
      STM32_EXTI_IMR_MR_SET(sink_id, x, 0);
#elif CONFIG_STM32_FAMILY == L4
      a = STM32_EXTI_ADDR + STM32_EXTI_IMR1_ADDR;
      x = endian_le32(cpu_mem_read_32(a));
      STM32_EXTI_IMR1_IM_SET(sink_id, x, 0);
#endif
      cpu_mem_write_32(a, endian_le32(x));
      return;
    }

  /* Set trigger. */
#if CONFIG_STM32_FAMILY == 4
  a = STM32_EXTI_ADDR + STM32_EXTI_FTSR_ADDR;
  x = endian_le32(cpu_mem_read_32(a));
  STM32_EXTI_FTSR_TR_SET(sink_id, x, f);
#elif CONFIG_STM32_FAMILY == L4
  a = STM32_EXTI_ADDR + STM32_EXTI_FTSR1_ADDR;
  x = endian_le32(cpu_mem_read_32(a));
  STM32_EXTI_FTSR1_FT_SET(sink_id, x, f);
#endif
  cpu_mem_write_32(a, endian_le32(x));

#if CONFIG_STM32_FAMILY == 4
  a = STM32_EXTI_ADDR + STM32_EXTI_RTSR_ADDR;
  x = endian_le32(cpu_mem_read_32(a));
  STM32_EXTI_RTSR_TR_SET(sink_id, x, r);
#elif CONFIG_STM32_FAMILY == L4
  a = STM32_EXTI_ADDR + STM32_EXTI_RTSR1_ADDR;
  x = endian_le32(cpu_mem_read_32(a));
  STM32_EXTI_RTSR1_RT_SET(sink_id, x, r);
#endif
  cpu_mem_write_32(a, endian_le32(x));

  /* Enable interrupt. */
#if CONFIG_STM32_FAMILY == 4
  a = STM32_EXTI_ADDR + STM32_EXTI_IMR_ADDR;
  x = endian_le32(cpu_mem_read_32(a));
  STM32_EXTI_IMR_MR_SET(sink_id, x, 1);
#elif CONFIG_STM32_FAMILY == L4
  a = STM32_EXTI_ADDR + STM32_EXTI_IMR1_ADDR;
  x = endian_le32(cpu_mem_read_32(a));
  STM32_EXTI_IMR1_IM_SET(sink_id, x, 1);
#endif
  cpu_mem_write_32(a, endian_le32(x));
}

static
DEV_ICU_GET_SINK(stm32_gpio_icu_get_sink)
{
  struct device_s             *dev = accessor->dev;
  struct stm32_gpio_private_s *pv  = dev->drv_pv;

  uint_fast8_t sink_id = id % STM32_GPIO_BANK_SIZE;
  uint_fast8_t bank    = id / STM32_GPIO_BANK_SIZE;

  if (bank > 8)
    return NULL;

  struct dev_irq_sink_s *sink = &pv->sink[sink_id];

  /* We actually keep track of only one endpoint per interrupt for all banks.
     We have to keep track of the bank number which the endpoint is linked to.
   */
  if (sink->base.link_count == 0)
    pv->irq[sink_id].bank = bank;
  else if(pv->irq[sink_id].bank != bank)
    return NULL;

  return sink;
}

static
DEV_ICU_LINK(stm32_gpio_icu_link)
{
  if (!route_mask || *bypass)
    return 0;

  struct device_s             *dev = accessor->dev;
  struct stm32_gpio_private_s *pv  = dev->drv_pv;

  uint_fast8_t sink_id = sink - pv->sink;
  uint_fast8_t bank    = pv->irq[sink_id].bank;

  gpio_id_t io_id = bank * STM32_GPIO_BANK_SIZE + sink_id;
  uint8_t mask[1] = { 0x1 };

  error_t err = 0;

  /* Change pin mode to input. */
  err = stm32_gpio_apply_mode(io_id, io_id, mask, DEV_PIN_INPUT, 0 /* alt */);
  if (err)
    return err;

  /* Setup link between the pin (i.e. bank) and and the external interrupt
     line. */
  uintptr_t a = STM32_SYSCFG_ADDR + STM32_SYSCFG_EXTICR_ADDR(sink_id >> 2);
  uint32_t  x = endian_le32(cpu_mem_read_32(a));
  STM32_SYSCFG_EXTICR_EXTI_SET(sink_id & 0x3, x, bank);
  cpu_mem_write_32(a, endian_le32(x));

  /* Clear interrupt. */
#if CONFIG_STM32_FAMILY == 4
  a = STM32_EXTI_ADDR + STM32_EXTI_PR_ADDR;
#elif CONFIG_STM32_FAMILY == L4
  a = STM32_EXTI_ADDR + STM32_EXTI_PR1_ADDR;
#endif
  x = endian_le32(cpu_mem_read_32(a));
  x &= ~(1 << sink_id);
  cpu_mem_write_32(a, endian_le32(x));

  return 0;
}

static
DEV_IRQ_SRC_PROCESS(stm32_gpio_icu_src_process)
{
  struct stm32_gpio_private_s *pv = ep->base.dev->drv_pv;

  while (1)
    {
#if CONFIG_STM32_FAMILY == 4
      uintptr_t a = STM32_EXTI_ADDR + STM32_EXTI_PR_ADDR;
#elif CONFIG_STM32_FAMILY == L4
      uintptr_t a = STM32_EXTI_ADDR + STM32_EXTI_PR1_ADDR;
#endif
      uint32_t  x = cpu_mem_read_32(a);

      if (!x)
        break;

      /* clear interrupts. */
      cpu_mem_write_32(a, x);

      x = endian_le32(x);
      while (x)
        {
          uint_fast8_t          sink_id = bit_ctz(x);
          struct dev_irq_sink_s *sink   = &pv->sink[sink_id];
          dev_irq_id_t          irq_id  = stm32_gpio_icu_src_id_of_sink_id(sink_id);
          device_irq_sink_process(sink, irq_id);
          x ^= 1 << sink_id;
        }
    }
}

#endif

/********************************* DRIVER */

#define stm32_gpio_gpio_cancel (dev_gpio_cancel_t*)&dev_driver_notsup_fcn
#define stm32_gpio_gpio_request dev_gpio_request_async_to_sync
#define stm32_gpio_gpio_input_irq_range (dev_gpio_input_irq_range_t*)dev_driver_notsup_fcn

#define stm32_gpio_use dev_use_generic

static DEV_INIT(stm32_gpio_init)
{
  struct stm32_gpio_private_s *pv = NULL;

#if defined(CONFIG_DRIVER_STM32_GPIO_X4_ICU)
  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;
  memset(pv, 0, sizeof(*pv));
#endif

  /* enable high-speed i/o on all ports. */
  uint8_t   bi;
  uintptr_t a = STM32_GPIO_ADDR;
  for (bi = 0; bi < 5; ++bi)
    cpu_mem_write_32(a + STM32_GPIO_OSPEEDR_ADDR(bi), endian_le32(-1));

#if defined(CONFIG_DRIVER_STM32_GPIO_X4_ICU)
  device_irq_source_init(dev, pv->src, STM32_GPIO_IRQ_SRC_COUNT,
    &stm32_gpio_icu_src_process);

  if (device_irq_source_link(dev, pv->src, STM32_GPIO_IRQ_SRC_COUNT, -1))
    goto err_mem;

  device_irq_sink_init(dev, pv->sink, STM32_GPIO_BANK_SIZE,
    &stm32_gpio_icu_sink_update, DEV_IRQ_SENSE_ANY_EDGE |
    DEV_IRQ_SENSE_FALLING_EDGE | DEV_IRQ_SENSE_RISING_EDGE);
#endif

  dev->drv_pv = pv;
  return 0;

err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(stm32_gpio_cleanup)
{
  struct stm32_gpio_private_s *pv = dev->drv_pv;

#if defined(CONFIG_DRIVER_STM32_GPIO_X4_ICU)
  device_irq_source_unlink(dev, pv->src, STM32_GPIO_IRQ_SRC_COUNT);
#endif

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(stm32_gpio_drv, 0, "STM32 GPIO", stm32_gpio,
               DRIVER_GPIO_METHODS(stm32_gpio_gpio),
#if defined(CONFIG_DRIVER_STM32_GPIO_X4_ICU)
               DRIVER_ICU_METHODS(stm32_gpio_icu),
#endif
               DRIVER_IOMUX_METHODS(stm32_gpio_iomux));

DRIVER_REGISTER(stm32_gpio_drv);

